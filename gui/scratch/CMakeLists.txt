project (gui-scratch)

# for <gui/core/...>
include_directories (${guis_SOURCE_DIR})

set (
    scratch_srcs
    main.cpp
    viewWidget.cpp
    ../gui/core/framework/elements/multiArc.cpp
    ../gui/core/framework/elements/multiArcStart.cpp
    ../gui/core/framework/elements/multiArcEnd.cpp
    ../gui/core/framework/elements/fNode.cpp
    ../gui/core/framework/elements/nodeInput.cpp

    ../gui/core/framework/fluidity/managedElement.cpp
    ../gui/core/framework/fluidity/manager.cpp

)

set (
    scratch_H
    viewWidget.h
    cross.h
    ../gui/core/framework/elements/multiArc.h
    ../gui/core/framework/elements/multiArcStart.h
    ../gui/core/framework/elements/multiArcEnd.h
    ../gui/core/framework/elements/fNode.h

    ../gui/core/framework/fluidity/manager.h
)


set (
    scratch_RCC

    ../gui/resources.qrc
)

qt4_wrap_cpp (scratch_MOC_H ${scratch_H})
qt4_add_resources(scratch_RCC_H ${scratch_RCC})


#set (
#    scratch_UIS
#    scratch.ui
#)

#qt4_wrap_ui (scratch_UIS_H ${scratch_UIS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../gui/core/framework/)


# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})

add_executable (
    scratch
    ${scratch_srcs}
    ${scratch_H}
    ${scratch_MOC_H}
    ${scratch_RCC_H}
)

target_link_libraries (
    scratch
    cauv_debug
    liquid
    ${QT_LIBRARIES}
)
#cauv_install ( scratch )

