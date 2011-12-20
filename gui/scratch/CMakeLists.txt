project (gui-scratch)

# for <gui/core/...>
include_directories (${guis_SOURCE_DIR})
include_directories (${guis_SOURCE_DIR}/gui/core/)

set (
    scratch_srcs
    main.cpp
    ../gui/core/framework/style.cpp

    ../gui/core/framework/fluidity/fNode.cpp
    ../gui/core/framework/fluidity/imgNode.cpp
    ../gui/core/framework/fluidity/managedElement.cpp
    ../gui/core/framework/fluidity/manager.cpp
    ../gui/core/framework/fluidity/fNodeInput.cpp
    ../gui/core/framework/fluidity/view.cpp
)

set (
    scratch_H
    ../gui/core/framework/style.h

    ../gui/core/framework/fluidity/fNode.h
    ../gui/core/framework/fluidity/fNodeOutput.h
    ../gui/core/framework/fluidity/fNodeInput.h
    ../gui/core/framework/fluidity/imgNode.h
    ../gui/core/framework/fluidity/manager.h
    ../gui/core/framework/fluidity/view.h
    ../gui/core/framework/fluidity/menu.h
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
    common
    liquid
    ${QT_LIBRARIES}
)
#cauv_install ( scratch )

depends_on_messages(scratch)
