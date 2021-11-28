/*
 * Uses fragments of code from oscfile/oscrec/oscplay
 * Copyright (c) 2012 Hanspeter Portner (agenthp@users.sf.net)
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <time.h>
#include <math.h>

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "lms.h"

const struct timespec _10mu = {
  .tv_sec = 0,
  .tv_nsec = 1e4
};


volatile int done = 0;

int noisy = 0;
int onoisy = 0;
int oonoisy = 0;

float leftangle  = 0.0;
float rightangle = 0.0;
float separation = 0.0;

static void _quit (int signal)
{
  done = 1;
}

static void _error (int num, const char *msg, const char *where)
{
  fprintf (stderr, "lo server error #%i '%s' at %s\n", num, msg, where);
}


int is_bundle = 0;
lo_bundle bundle = NULL;

static int _bundle_start_handler (lo_timetag time, void *data)
{
  // printf("bundle start\n");

  if ( (time.sec == LO_TT_IMMEDIATE.sec) && (time.frac == LO_TT_IMMEDIATE.frac) )
		lo_timetag_now (&time);

  bundle = lo_bundle_new (time); //FIXME handle nested bundles
  is_bundle++;
}

static int _bundle_end_handler (void *data)
{
  lo_address addr = data;
	
  // printf("bundle end buf %s\n",buf);

  lo_send_bundle (addr, bundle);
  lo_bundle_free_messages (bundle);

  bundle = NULL;
  is_bundle--;
}

// handles both outl and outb by passing them on to endpoint

static int outl_msg_handler (const char *path, const char *types, lo_arg **argv, int argc, lo_message msg, void *data)
{
  lo_message _msg = msg;
  lo_address addr = data;
	
  if (is_bundle) {
    _msg = lo_message_clone (msg);    
    lo_bundle_add_message (bundle, path, _msg);
  }

  else { // (!is_bundle) {
    lo_send_message(addr,path,msg);
  }

  return 0;
}


static int ob_msg_forward (const char *path, lo_message msg, void *data)
{
  lo_message _msg = msg;
  lo_address addr = data;
	

  // forward object messages to endpoint
  
  if (is_bundle) {
    _msg = lo_message_clone (msg);
    lo_bundle_add_message (bundle, path, _msg);
  }

  else { //  (!is_bundle) {
    lo_send_message(addr,path,msg);
  }
  return 0;
}

static int ob_msg_handler (const char *path, const char *types, lo_arg **argv, int argc, lo_message msg, void *data)
{
  // expecting ciffiii : sensor, object, birth, lifetime, alive, valid, zombie

  if (oonoisy) {
    if (argc >= 5) {
      printf("properties : sensor %c ob %d from %f for %f : %d %d %d\n",
	     argv[0]->c, argv[1]->i, argv[2]->f,  argv[3]->f,  argv[4]->i,  argv[5]->i, argv[6]->i);
    }
  }

  update_props(argv[0]->c, argv[1]->i, argv[2]->f,  argv[3]->f,  argv[4]->i,  argv[5]->i, argv[6]->i);

  return ob_msg_forward (path, msg, data);
}

static int obC_msg_handler (const char *path, const char *types, lo_arg **argv, int argc, lo_message msg, void *data)
{
  // expecting ciffff : sensor, object, x, y, width, depth

  if (oonoisy) {
    if (argc >= 5) {
      printf("cartesian : sensor %c ob %d at %f, %f size %fm %fm\n",
	     argv[0]->c, argv[1]->i, argv[2]->f,  argv[3]->f,  argv[4]->f,  argv[5]->f);
    }
  }
  return 0; // ob_msg_forward (path, msg, data);
}

static int obP_msg_handler (const char *path, const char *types, lo_arg **argv, int argc, lo_message msg, void *data)
{
  // expecting ciffff : sensor, object, angle, distance, width, depth

  if (onoisy) {
    if (argc >= 5) {
      printf("polar : sensor %c ob %d at %f deg, %fm size %fm %fm\n",
	     argv[0]->c, argv[1]->i, argv[2]->f,  argv[3]->f,  argv[4]->f,  argv[5]->f);
    }
  }

  update_polar(argv[0]->c, argv[1]->i, argv[2]->f,  argv[3]->f,  argv[4]->f,  argv[5]->f);
  
  return ob_msg_forward (path, msg, data);
}


static int default_msg_handler (const char *path, const char *types, lo_arg **argv, int argc, lo_message msg, void *data)
{
  // report unhandled message

  if (noisy) {
    printf("forwarding unexpected msg %s, type %s\n", path, types);
  }
  return ob_msg_forward (path, msg, data);
}




int main (int argc, char **argv)
{
  lo_address addr = NULL;
  lo_server_thread serv = NULL;
  FILE *file = NULL;
  int c, proto;
  float angle;
  
  while ( (c = getopt (argc, argv, "i:o:dDl:r:s:")) != -1)
    switch (c) {

    case 'i':
      proto = lo_url_get_protocol_id (optarg);
      if (proto == -1)
	fprintf (stderr, "protocol not supported\n");
      char *port = lo_url_get_port (optarg);
      serv = lo_server_thread_new_with_proto (port, proto, _error);
      free (port);
      break;
      
    case 'o':
      addr = lo_address_new_from_url (optarg);
      break;

    case 'd':
      noisy = !0;
      break;
      
    case 'D':
      onoisy = !0;
      break;
      
    case 'l':
      if (sscanf(optarg,"%f",&angle) == 1)
	leftangle = (2 * M_PI * angle)/360.0;
      break;
      
    case 'r':
      if (sscanf(optarg,"%f",&angle) == 1)
	rightangle = (2 * M_PI * angle)/360.0;
      break;
      
    case 's':
      sscanf(optarg,"%f",&separation);
      break;
      
    case '?':
	if ( (optopt == 'i') || (optopt == 'o') )
	  fprintf (stderr, "Option `-%c' requires an argument.\n", optopt);
	else if (isprint (optopt))
	  fprintf (stderr, "Unknown option `-%c'.\n", optopt);
	else
	  fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
	return 1;
      default:
	return (1);
      }
		
  if (!serv || !addr)
    {
      fprintf (stderr, "usage: %s -i PROTOCOL://[localhost]:PORT -o PROTOCOL://[host]:PORT\n\n", argv[0]);
      return (1);
    }

  lo_server *_serv = lo_server_thread_get_server (serv);
  lo_server_add_bundle_handlers (_serv, _bundle_start_handler, _bundle_end_handler, addr);
  lo_server_enable_queue (_serv, 0, 1);

  lo_server_thread_add_method (serv, "/obP", "ciffff",  obP_msg_handler,     addr);
  lo_server_thread_add_method (serv, "/obC", "ciffff",  obC_msg_handler,     addr);
  lo_server_thread_add_method (serv, "/ob",  "ciffiii", ob_msg_handler,      addr);
  lo_server_thread_add_method (serv, "/outl", NULL,     outl_msg_handler,    addr);
  lo_server_thread_add_method (serv, "/outb", NULL,     outl_msg_handler,    addr);
  lo_server_thread_add_method (serv,  NULL,   NULL,     default_msg_handler, addr);

  lo_server_thread_start (serv);
  
  signal (SIGHUP, _quit);
  signal (SIGQUIT, _quit);
  signal (SIGTERM, _quit);
  signal (SIGINT, _quit);
  
  while (!done)
    nanosleep (&_10mu, NULL);
  
  fprintf (stderr, "lmsconc finishing\n");
  
  lo_server_thread_stop (serv);
  lo_server_thread_free (serv);
  
  return 0;
}
