project (gui)

find_package (OIS REQUIRED)
find_package (X11 REQUIRED)

set ( GAMEPAD_PLUGIN_SRCS
    plugin.cpp
    cauvgamepad.cpp
    gamepad/gamepadinput.cpp
    gamepad/playstationinput.cpp
    gamepad/xboxinput.cpp
)

set ( GAMEPAD_PLUGIN_UIS
)

set ( GAMEPAD_PLUGIN_H
    plugin.h
    cauvgamepad.h
    gamepad/gamepadinput.h
    gamepad/xboxinput.h
    gamepad/playstationinput.h
)

     
qt4_wrap_ui (GAMEPAD_PLUGIN_UIS_H ${GAMEPAD_PLUGIN_UIS})
qt4_wrap_cpp (GAMEPAD_PLUGIN_MOC_H ${GAMEPAD_PLUGIN_H})

add_library( gamepadplugin MODULE
    ${GAMEPAD_PLUGIN_SRCS}
    ${GAMEPAD_PLUGIN_UIS_H}
    ${GAMEPAD_PLUGIN_MOC_H}
)
cauv_install_plugin( gamepadplugin )
                                      
target_link_libraries( gamepadplugin
    gui-core
    ${X11_X11_LIB}
    ${QT_LIBRARIES}
    ${OIS_LIBRARIES}
)
