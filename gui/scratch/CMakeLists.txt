project (gui-scratch)


include_directories(${guis_SOURCE_DIR})
include_directories(${guis_SOURCE_DIR}/gui/core/)
include_directories(${guis_SOURCE_DIR}/gui/core/framework)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set (
    scratch_srcs
    main.cpp
)

set (
    scratch_H
)


set (
    scratch_RCC
)

qt4_wrap_cpp (scratch_MOC_H ${scratch_H})
qt4_add_resources(scratch_RCC_H ${scratch_RCC})


#set (
#    scratch_UIS
#    scratch.ui
#)

#qt4_wrap_ui (scratch_UIS_H ${scratch_UIS})

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
    gui-core
)
#cauv_install ( scratch )

depends_on_messages(scratch)
