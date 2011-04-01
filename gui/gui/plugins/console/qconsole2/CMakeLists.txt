#find_package (Qt3)

#include_directories(${QT_INCLUDE_DIR})

add_definitions(-DQT3_SUPPORT)
include_directories( ${QT_QT3SUPPORT_INCLUDE_DIR} )

set (
	QCONSOLE2_SRCS        
    	
	src/qconsole.cpp
    	src/interceptor.cpp
)

set (
        QCONSOLE2_H
                
	include/qconsole.h
    	include/interceptor.h
)

include_directories (include)

qt4_wrap_cpp (QCONSOLE2_MOC_H ${QCONSOLE2_H})

add_library (
	qconsole2 STATIC

	${QCONSOLE2_SRCS}
	${QCONSOLE2_MOC_H}
)

target_link_libraries (
    qconsole2

    ${QT_QT3SUPPORT_LIBRARY}
)
