# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#


set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
add_custom_command ( 
       OUTPUT pinger
       WORKING_DIRECTORY ${SRCD}
       COMMAND make
       COMMAND cp pinger ${CMAKE_CURRENT_BINARY_DIR}/
       DEPENDS ${SRCD}/Makefile
               ${SRCD}/JackMM.cpp
               ${SRCD}/JackMM.h
               ${SRCD}/PingerListenD.cpp
               ${SRCD}/Exception.h
       COMMENT "building pinger with make"
)


