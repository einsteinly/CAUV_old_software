project (gui-core)

# required packages
find_package (Qt4 REQUIRED)
find_package (OpenCV REQUIRED)


set ( GUI_CORE_AUX_DIRS
    framework
    controller
    model
    utils
    widgets
)

# include files
include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/framework)

# generated files includes
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/framework)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/widgets)

# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})

# for <gui/core/...>
include_directories (${guis_SOURCE_DIR})

set ( GUI_CORE_SRCS
    nodedragging.cpp
    cauvbasicplugin.cpp

    widgets/videoscreen.cpp
    widgets/neutralspinbox.cpp
    widgets/graphbar.cpp

    model/node.cpp
    model/variants.cpp
    model/model.cpp
    model/registry.cpp
    model/messageobserver.cpp
    model/messagegenerators.cpp
    model/paramvalues.cpp
    model/utils/datarecorder.cpp
    model/utils/sampler.cpp

    framework/nodepicker.cpp
    framework/mainwindow.cpp
    framework/nodescene.cpp
    framework/style.cpp
    framework/delegates.cpp

    framework/fluidity/fNode.cpp
    framework/fluidity/imgNode.cpp
    framework/fluidity/view.cpp 
    framework/fluidity/managedElement.cpp
    framework/fluidity/manager.cpp
    framework/fluidity/fNodeInput.cpp
    )

set ( GUI_CORE_UIS
    widgets/videoscreen.ui
    framework/nodepicker.ui
    framework/mainwindow.ui
    )

set ( GUI_CORE_H
    nodedragging.h

    widgets/videoscreen.h
    widgets/neutralspinbox.h
    widgets/graphbar.h

    model/node.h
    model/model.h
    model/nodes/numericnode.h
    model/nodes/stringnode.h
    model/nodes/imagenode.h
    model/nodes/vehiclenode.h
    model/messagegenerators.h
    model/utils/sampler.h

    framework/nodepicker.h
    framework/mainwindow.h
    framework/style.h
    framework/delegates.h

    framework/fluidity/view.h
    framework/fluidity/manager.h
    framework/fluidity/fNode.h
    framework/fluidity/fNodeOutput.h
    framework/fluidity/fNodeInput.h
    framework/fluidity/imgNode.h
    framework/fluidity/menu.h
    )

set ( GUI_CORE_RCC
    ../resources.qrc
    )

qt4_wrap_ui (GUI_CORE_UIS_H ${GUI_CORE_UIS})
qt4_wrap_cpp (GUI_CORE_MOC_H ${GUI_CORE_H})
qt4_add_resources(GUI_CORE_RCC_H ${GUI_CORE_RCC})

add_library ( gui-core SHARED
        ${GUI_CORE_SRCS}
        ${GUI_CORE_UIS_H}
        ${GUI_CORE_MOC_H}
        ${GUI_CORE_RCC_H}
    )

cauv_install( gui-core )

target_link_libraries ( gui-core
        cauv_debug
        liquid
        common
        ${OpenCV_LIBS}
        ${QT_LIBRARIES}
    )
depends_on_messages(gui-core)
