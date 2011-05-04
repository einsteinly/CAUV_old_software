project (guicpp)

find_package (OIS REQUIRED)


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
                                      
target_link_libraries( gamepadplugin
    guicpp-core
    ${QT_LIBRARIES}
    ${OIS_LIBRARIES}
)
