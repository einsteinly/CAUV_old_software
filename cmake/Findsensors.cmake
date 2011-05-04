 # - Try to find the Sensors Library
# Once done this will define
#
#  sensors_FOUND - system has Sensors
#  sensors_INCLUDE_DIR - the Sensors include directory
#  sensors_LIBRARIES 
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#


if ( sensors_INCLUDE_DIR AND sensors_LIBRARIES )
   # in cache already
   SET( sensors_FIND_QUIETLY TRUE )
endif ( sensors_INCLUDE_DIR AND sensors_LIBRARIES )

FIND_PATH( sensors_INCLUDE_DIR NAMES sensors/sensors.h )

FIND_LIBRARY( sensors_LIB NAMES sensors )

SET( sensors_LIBRARIES ${sensors_LIB} )

include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( sensors DEFAULT_MSG sensors_INCLUDE_DIR sensors_LIBRARIES )

mark_as_advanced( sensors_INCLUDE_DIR sensors_LIB )



