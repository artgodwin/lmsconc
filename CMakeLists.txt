cmake_minimum_required (VERSION 2.8)

project (lmsconc)

set (LMSCONC_VERSION "0.1.0")

# pkg-config
find_package (PkgConfig) # ${PKG_CONFIG_FOUND}

set (LIBS "-lm")

# liblo
pkg_search_module (LIBLO REQUIRED liblo)
include_directories (${LIBLO_INCLUDE_DIRS})
set (LIBS ${LIBS} ${LIBLO_LDFLAGS})

# lmsconc
add_executable (lmsconc lmsconc.c objects.c lms.h)
target_link_libraries (lmsconc ${LIBS})
install (TARGETS lmsconc DESTINATION bin)


