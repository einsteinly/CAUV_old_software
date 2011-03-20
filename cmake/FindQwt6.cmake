# Find the Qwt 6.x includes and library, linked to Qt4
#
# On Windows it makes these assumptions:
#    - the Qwt DLL is where the other DLLs for Qt are (QT_DIR\bin) or in the path
#    - the Qwt .h files are in QT_DIR\include\Qwt or in the path
#    - the Qwt .lib is where the other LIBs for Qt are (QT_DIR\lib) or in the path
#
# Qwt6_INCLUDE_DIR - where to find qwt.h if Qwt
# Qwt6_LIBRARY - The Qwt6 library linked against Qt4 (if it exists)
# Qwt6_FOUND - Set to TRUE if Qwt6 was found (linked either to Qt4)

# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
# Modified 2010, Leszek Swirski <leszek@swirski.co.uk>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# Condition is "(A OR B) AND C", CMake does not support parentheses but it evaluates left to right
IF(Qwt6_LIBRARY AND Qwt6_INCLUDE_DIR)
    SET(Qwt6_FIND_QUIETLY TRUE)
ENDIF()

IF(NOT QT4_FOUND)
    IF (Qwt_REQUIRED)
        FIND_PACKAGE( Qt4 REQUIRED QUIET )
    ELSE()
        FIND_PACKAGE( Qt4 QUIET )
    ENDIF()
ENDIF(NOT QT4_FOUND)

IF( QT4_FOUND )
    # Is Qwt6 installed? Look for header files
    FILE(GLOB POSSIBLE_QWT6_DIRS /usr/local/qwt*
                     /usr/qwt*
    )
    FIND_PATH( Qwt6_INCLUDE_DIR
        qwt.h
        PATHS ${QT_INCLUDE_DIR}
              ${POSSIBLE_QWT6_DIRS}
              /usr/local
              /usr
              ENV PATH
        PATH_SUFFIXES include lib/qwt.framework/Headers
    )
    #message("Qwt6_INCLUDE_DIR = ${Qwt6_INCLUDE_DIR}")
    # Find Qwt version
    IF( Qwt6_INCLUDE_DIR )
        FILE( READ ${Qwt6_INCLUDE_DIR}/qwt_global.h QWT_GLOBAL_H )
        STRING( REGEX MATCH "#define *QWT_VERSION *(0x06*)" QWT_IS_VERSION_6 ${QWT_GLOBAL_H})

        IF( QWT_IS_VERSION_6 )
            STRING(REGEX REPLACE ".*#define[\\t\\ ]+QWT_VERSION_STR[\\t\\ ]+\"([0-9]+\\.[0-9]+\\.[0-9]+[^\"]*)\".*" "\\1" Qwt_VERSION "${QWT_GLOBAL_H}")

            # Find Qwt6 library linked to Qt4
            FIND_LIBRARY( Qwt6_LIBRARY
                NAMES qwt6-qt4 qwt-qt4 qwt6 qwt
                PATHS ${Qwt6_INCLUDE_DIR}/../lib
                  ${QT_INCLUDE_DIR}
                  ${POSSIBLE_QWT6_DIRS}
                  /usr/local
                  /usr
                PATH_SUFFIXES lib
            )
            IF( Qwt6_LIBRARY )
                IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
                    EXECUTE_PROCESS( COMMAND "otool" "-L" "${Qwt6_LIBRARY}" OUTPUT_VARIABLE Qwt_Qt4_LIBRARIES_LINKED_TO )
                    #MESSAGE("Qwt linked to: ${Qwt_Qt4_LIBRARIES_LINKED_TO}")
                    STRING( REGEX MATCH ".*QtCore.*" Qwt6_IS_LINKED_TO_Qt4 ${Qwt_Qt4_LIBRARIES_LINKED_TO})
                ELSEIF(UNIX AND NOT CYGWIN)
                    EXECUTE_PROCESS( COMMAND "ldd" ${Qwt6_LIBRARY} OUTPUT_VARIABLE Qwt_Qt4_LIBRARIES_LINKED_TO )
                    STRING( REGEX MATCH ".*QtCore.*" Qwt6_IS_LINKED_TO_Qt4 ${Qwt_Qt4_LIBRARIES_LINKED_TO})
                ELSE()
                    # Assumes qwt.dll is in the Qt dir
                    SET(Qwt6_IS_LINKED_TO_Qt4 TRUE)
                ENDIF()
                        
                IF( Qwt6_IS_LINKED_TO_Qt4 )
                    SET( Qwt6_FOUND TRUE )
                    IF (NOT Qwt6_FIND_QUIETLY)
                        MESSAGE( STATUS "Found Qwt: ${Qwt6_LIBRARY}" )
                    ENDIF (NOT Qwt6_FIND_QUIETLY)
                ENDIF()
            ENDIF()

        ENDIF()

        MARK_AS_ADVANCED( Qwt6_INCLUDE_DIR Qwt6_LIBRARY )
    ENDIF( Qwt6_INCLUDE_DIR )

    IF (NOT Qwt6_FOUND AND Qwt6_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find Qwt 6.x")
    ENDIF (NOT Qwt6_FOUND AND Qwt6_FIND_REQUIRED)

    set(Qwt6_INCLUDE_DIRS ${Qwt6_INCLUDE_DIR} ${QT4_INCLUDE_DIRS})
    set(Qwt6_LIBRARIES ${Qwt6_LIBRARY} ${QT4_LIBRARIES})

ENDIF( QT4_FOUND )
