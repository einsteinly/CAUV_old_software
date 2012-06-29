project (gui)

# main GUI options
option(CAUV_GUI_MAP "True if the GUI should have a map of the Earth" ON)
option(CAUV_GUI_GAMEPAD "True if the GUI should support game controllers" ON)


# includes for shared and generated files
include_directories (${guis_SOURCE_DIR})
include_directories (${AUV_SOURCE_DIR})

get_property (gui_RUNTIME_OUTPUT_DIRECTORY TARGET gui PROPERTY RUNTIME_OUTPUT_DIRECTORY)
SET(LIBRARY_OUTPUT_PATH ${gui_RUNTIME_OUTPUT_DIRECTORY}/plugins)

function (cauv_install_plugin TARGET)
    install (TARGETS ${TARGET} LIBRARY DESTINATION bin/cauv/${CMAKE_BUILD_TYPE}/plugins )
endfunction()

# plugin directories
#add_subdirectory (example)
#add_subdirectory (console)
#add_subdirectory (pipeline)
#add_subdirectory (processes)
#add_subdirectory (motorcontrol)
add_subdirectory (ai)
add_subdirectory (fluidity)
add_subdirectory (redherring)
#add_subdirectory (qwtgraphs)
#add_subdirectory (logs)

#if (CAUV_GUI_MAP)
#    add_subdirectory (map)
#endif()

#if (CAUV_GUI_GAMEPAD)
#    add_subdirectory (gamepad)
#endif()
