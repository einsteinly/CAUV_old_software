project (gui-core)

# required packages
find_package (Qt4 REQUIRED)
find_package (Qwt6 REQUIRED)
include_directories (SYSTEM ${Qwt6_INCLUDE_DIRS})
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
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/controller)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/framework)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/model)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/widgets)

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
    widgets/graph.cpp
    model/node.cpp
    model/variants.cpp
    model/model.cpp
    controller/messageobserver.cpp
    controller/messagegenerators.cpp
    framework/nodepicker.cpp
    framework/treeitems.cpp
    framework/mainwindow.cpp
    framework/nodescene.cpp
    framework/elements/arc.cpp
    framework/elements/aiNode.cpp
    #framework/elements/nodeInput.cpp
    utils/noderecorder.cpp
    
    framework/fluidity/fNode.cpp
    framework/fluidity/imgNode.cpp
    framework/fluidity/view.cpp 
    framework/fluidity/managedElement.cpp
    framework/fluidity/manager.cpp
    )

set ( GUI_CORE_UIS
    widgets/videoscreen.ui
    framework/nodepicker.ui
    framework/mainwindow.ui
    widgets/graph.ui
    )

set ( GUI_CORE_H
    nodedragging.h
    widgets/videoscreen.h
    widgets/graph.h
    model/node.h
    model/model.h
    model/nodes/numericnode.h
    model/nodes/compoundnodes.h
    model/nodes/stringnode.h
    model/nodes/imagenode.h
    model/nodes/groupingnode.h
    controller/messagegenerators.h
    framework/nodepicker.h
    framework/treeitems.h
    framework/mainwindow.h
    framework/elements/arc.h
    framework/elements/aiNode.h
    #framework/elements/nodeInput.h

    framework/fluidity/view.h
    framework/fluidity/manager.h
    framework/fluidity/fNode.h
    framework/fluidity/imgNode.h
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
        ${Qwt6_LIBRARIES}
        ${OpenCV_LIBS}
        ${QT_LIBRARIES}
    )
