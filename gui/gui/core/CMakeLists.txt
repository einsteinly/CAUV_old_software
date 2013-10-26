# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (gui-core)

# required packages
find_package (Qt4 REQUIRED)

set ( GUI_CORE_AUX_DIRS
    framework
    controller
    model
    utils
    widgets
)

# include files
include_directories(${CMAKE_CURRENT_SOURCE_DIR} )

# generated files includes
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/widgets)

# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})

# for <gui/core/...>
include_directories (${guis_SOURCE_DIR})

set ( GUI_CORE_SRCS
    cauvbasicplugin.cpp
    nodepicker.cpp
    mainwindow.cpp
    nodescene.cpp
    connectednode.cpp

    widgets/neutralspinbox.cpp
    widgets/onoff.cpp
    widgets/videoWidget.cpp

    model/node.cpp
    model/variants.cpp
    model/nodeItemModel.cpp
    model/registry.cpp
    model/messaging.cpp
    model/paramvalues.cpp

    styles/style.cpp

    delegates/delegate.cpp
    delegates/booleandelegate.cpp
    delegates/numericdelegate.cpp
    delegates/colourdelegate.cpp

    elements/style.cpp

    trees/nodeTreeView.cpp

    filter/nodePathFilter.cpp
    filter/nodeExclusionFilter.cpp

    drag/graphDropHandler.cpp
    drag/groupDropHandler.cpp
    drag/nodeDragging.cpp
    )

set ( GUI_CORE_UIS
    nodepicker.ui
    mainwindow.ui
    )

set ( GUI_CORE_H

    nodepicker.h
    mainwindow.h

    widgets/neutralspinbox.h
    widgets/onoff.h
    widgets/videoWidget.h

    model/node.h
    model/nodeItemModel.h
    model/registry.h
    model/nodes/numericnode.h
    model/nodes/stringnode.h
    model/nodes/imagenode.h
    model/nodes/vehiclenode.h
    model/nodes/autopilotnode.h
    model/nodes/sonarnode.h
    model/nodes/colournode.h
    model/messaging.h

    styles/style.h

    delegates/delegate.h
    delegates/booleandelegate.h
    delegates/numericdelegate.h
    delegates/colourdelegate.h

    trees/nodeTreeView.h

    filter/nodePathFilter.h
    filter/nodeExclusionFilter.h

    drag/nodeDragging.h
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

install(TARGETS gui-core
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

target_link_libraries ( gui-core
        cauv_debug
        liquid
        pthread
        ${QT_LIBRARIES}
    )
