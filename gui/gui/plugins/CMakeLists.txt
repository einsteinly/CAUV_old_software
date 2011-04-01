project (guicpp)

# main GUI options
option(CAUV_GUI_MAP "True if the GUI should have a map of the Earth. For some reason." ON)
option(CAUV_GUI_GAMEPAD "True if the GUI should support game controllers" ON)


# includes for shared and generated files
include_directories(${guicpp_BINARY_DIR})
include_directories (${gui_SOURCE_DIR})
include_directories (${AUV_SOURCE_DIR})


# plugin directories
add_subdirectory (example)
add_subdirectory (datastreamdisplay)

if (CAUV_GUI_MAP)
    add_subdirectory (map)
endif()

if (CAUV_GUI_GAMEPAD)
    add_subdirectory (gamepad)
endif()
