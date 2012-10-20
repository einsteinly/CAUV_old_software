project (gui-fridgetest)


include_directories(${guis_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set (
    fridgetest_srcs
    main.cpp
)

set (
    fridgetest_H
)


set (
    fridgetest_RCC
    ../gui/resources.qrc    
)

qt4_wrap_cpp (fridgetest_MOC_H ${fridgetest_H})
qt4_add_resources(fridgetest_RCC_H ${fridgetest_RCC})


#set (
#    fridgetest_UIS
#    fridgetest.ui
#)

#qt4_wrap_ui (fridgetest_UIS_H ${fridgetest_UIS})

# for <common/includesstuff> etc
#include_directories (${AUV_SOURCE_DIR})

add_executable (
    fridgetest
    ${fridgetest_srcs}
    ${fridgetest_H}
    ${fridgetest_MOC_H}
    ${fridgetest_RCC_H}
)

target_link_libraries (
    fridgetest
    liquid
    #common
    #gui-core
)
#cauv_install ( fridgetest )

#depends_on_messages(fridgetest)
