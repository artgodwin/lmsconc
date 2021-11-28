#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "lms.h"

#define SOURCE_OBJECTS  20
#define MAP_OBJECTS     8

// bitmap to label the sources of data valid for objects in the output table
enum sources { NONE, LEFT=1, RIGHT=2, ESTIMATE=4 };

struct obpolar {
  int source;
  struct obcart *destp;
  float birth;
  float lifetime;
  int alive;
  int valid;
  float match;  // least-squares value for succesful output object
  float ux;     // coordinates converted to output cartesian system
  float uy;
  float angle;  // coordinates provided in source's polar system
  float dist;
  float width;  // size in linear dimensions (not angular width) as seen at reported angle
  float depth;  // ie width is normal to angular radius, depth is parallel
} left[SOURCE_OBJECTS], right[SOURCE_OBJECTS];

struct obcart {
  int sources;
  struct obpolar *sourcel;
  struct obpolar *sourcer;
  float birth;
  float lifetime;
  int alive;
  int valid;
  float ux;
  float uy;
  float width;
  float depth;
  //float speed;
  //float dir;
  //int nearest;
  //int crowding;
} out[MAP_OBJECTS];

// make output object reusable by removing links

void erase(struct obcart *p) {
  
    if (p->sourcel) p->sourcel->destp = NULL;
    if (p->sourcer) p->sourcer->destp = NULL;
    p->sourcel = NULL;
    p->sourcer = NULL;
    p->sources = NONE;
}

// convert from polar sensor coordinates to cartesian map coordinates
// for left, x is -ve sep getting more -ve as dist is small
// for right, x is +ve sep getting more +ve as dist is small
void ptoc(struct obpolar *p, float sangle, float sep, float dir)
{
  p->uy = sin(dir*(p->angle - M_PI_2) + sangle) * p->dist;
  p->ux = sep+(dir*(cos(dir*(p->angle - M_PI_2) + sangle) * p->dist));

  if (onoisy)
    printf("ptoc %f (%f) by %f at %f sep %f, dir %f\n",
	   p->angle * 360.0 / (M_PI * 2),
	   dir*(p->angle - M_PI_2) * 360.0 / (M_PI * 2),
	   sangle * 360.0 / (M_PI * 2), p->dist, sep, dir); 

  if (onoisy)
    printf("ux angle %f cos %f * %f -> %f in %f -> %f\n",
	   (dir*(p->angle - M_PI_2) + sangle)*360.0/(M_PI*2),
	   cos(dir*(p->angle - M_PI_2) + sangle),
	   p->dist,
	   cos(dir*(p->angle - M_PI_2) + sangle) * p->dist,
	   sep, p->ux);

  if (onoisy)
    printf("uy angle %f sin %f * %f -> %f\n",
	   (dir*(p->angle - M_PI_2) + sangle)*360.0/(M_PI*2),
	   sin(dir*(p->angle - M_PI_2) + sangle),
	   p->dist, p->uy);
}

// calculate a match metric for this sensor object against a map object - smaller is better

#define WORST_MATCH  0.2   // limit of sum-of-squares : worse than this is not a match

float match(struct obcart *op, struct obpolar *pp)
{
  if (onoisy)
    printf("match output %ld %f,%f against %f,%f\n",
	   op - out, op->ux,op->uy, pp->ux,pp->uy); 
  
  float x = op->ux - pp->ux;
  float y = op->uy - pp->uy;

  return x*x + y*y;
}


// match the reported object against existing list

void update_map(struct obpolar *p)
{
  struct obcart *dp = NULL;
  static struct obcart *next = out;
  float try, best = 0;

  // need to find match ?
  if (p->destp == NULL) {

    if (onoisy)
      printf("find near to %f,%f\n", p->ux,p->uy);

    // find nearest existing object
    for (struct obcart *bp = out; bp < out + MAP_OBJECTS; ++bp) {
      if (bp->valid)
	if ( ((try = match(bp,p)) < best) || (best == 0)) {
	  dp = bp;
	  best = try;
	  if (onoisy)
	    printf("try %f\n", try);
	}
    }
    // if that isn't near enough, discard it
    if (best > WORST_MATCH) {
      if (onoisy)
	printf("%f too far away\n", best);
      dp = NULL;
    }
  }

  // confirm match
  else {
    // previously-found match is still good enough
    // should probably re-search if this fails
    if ((try = match(p->destp,p)) < WORST_MATCH) {
      dp = p->destp;
      if (onoisy)
	printf("match confirmed %f\n", try);
    }
    else if (onoisy)
      printf("match not confirmed %f\n", try);
  }

  // allocate new slot
  if (dp == NULL) {
    dp = next++;
    if (onoisy)
      printf("new slot %ld\n", dp - out);
    // wrapped around : at present just start at the bottom but needs more thought
    if (next >= out + MAP_OBJECTS)
      next = out;
    // ensure next entry is reusable
    erase(next);
  }
  
  p->destp = dp;
  p->match = try;
  if (p->source & LEFT)  dp->sourcel = p;
  if (p->source & RIGHT) dp->sourcer = p;
  // smooth new values into combined object
  dp->ux = (dp->ux * 0.7) + (p->ux * 0.3);
  dp->uy = (dp->uy * 0.7) + (p->uy * 0.3);
  dp->width = (dp->width * 0.7) + (p->width * 0.3);
  dp->depth = (dp->depth * 0.7) + (p->depth * 0.3);
  dp->sources |= p->source;
  dp->birth    = p->birth;
  dp->lifetime = p->lifetime;
  dp->alive   |= p->alive;
  dp->valid   |= p->valid;
   
}


void update_props( char sensor, int ob, float birth, float life, int alive, int valid, int zombie)
{
  struct obpolar *p;
  switch(sensor) {
  case 'l':
    p = left;
    break;
  case 'r':
    p = right;
    break;
  default:
    return;   // ignore invalid source sensor
  }

  if (ob < SOURCE_OBJECTS) {
    p += ob;
    p->source   = sensor;
    p->birth    = birth;
    p->lifetime = life;
    p->alive    = alive;
    p->valid    = valid;
  }
}

void update_polar( char sensor, int ob, float angle, float dist, float width, float depth)
{
  struct obpolar *p;
  float sangle, distance, dir;
  int source = 0;
  
  switch(sensor) {
  case 'l':
    p = left;
    source = LEFT;
    sangle = leftangle;
    distance = -separation/2;
    dir = 1.0;
    break;
  case 'r':
    p = right;
    source = RIGHT;
    sangle = rightangle;
    distance = separation/2;
    dir = -1.0;
    break;
  default:
    return;   // ignore invalid source sensor
  }

  if (ob < SOURCE_OBJECTS) {
    p += ob;
    p->source = source;
    p->angle  = angle * 2 * M_PI / 360.0;
    p->dist   = dist;
    p->width  = width;
    p->depth  = depth;

    // convert sensor coordinates to map
    ptoc(p,sangle,distance,dir);

    if (onoisy) {
      printf("object from %c at %f deg, %fm ==> x %f, y %f\n", sensor, angle, dist, p->ux, p->uy);
    }

    // insert it into unified object list
    update_map(p);

    // send it out as osc
    
  }
}
