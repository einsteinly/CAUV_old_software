 # - Try to find the Avahi Library
# Once done this will define
#
#  AVAHI_FOUND - system has Avahi
#  AVAHI_INCLUDE_DIR - the Avahi include directory
#  AVAHI_LIBRARIES 
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#


if ( AVAHI_INCLUDE_DIR AND AVAHI_LIBRARIES )
   # in cache already
   SET( AVAHI_FIND_QUIETLY TRUE )
endif ( AVAHI_INCLUDE_DIR AND AVAHI_LIBRARIES )

FIND_PATH( AVAHI_INCLUDE_DIR NAMES avahi-core/core.h )

FIND_LIBRARY( AVAHI_CLIENT_LIB NAMES avahi-client )
FIND_LIBRARY( AVAHI_CORE_LIB NAMES avahi-core )
FIND_LIBRARY( AVAHI_COMMON_LIB NAMES avahi-common )

SET( AVAHI_LIBRARIES ${AVAHI_CLIENT_LIB} ${AVAHI_CORE_LIB} ${AVAHI_COMMON_LIB})

include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( avahi DEFAULT_MSG AVAHI_INCLUDE_DIR AVAHI_LIBRARIES )




