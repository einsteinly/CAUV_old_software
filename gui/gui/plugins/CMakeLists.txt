# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (plugins)

# includes for shared and generated files
include_directories (${guis_SOURCE_DIR})
include_directories (${gui-core_SOURCE_DIR})
include_directories (${plugins_SOURCE_DIR})

get_property (gui_RUNTIME_OUTPUT_DIRECTORY TARGET cauv_gui PROPERTY RUNTIME_OUTPUT_DIRECTORY)
SET(LIBRARY_OUTPUT_PATH ${gui_RUNTIME_OUTPUT_DIRECTORY}/plugins)

function (cauv_install_plugin TARGET)
    install (TARGETS ${TARGET} LIBRARY DESTINATION bin/${CAUV_INSTALL_DIR}/plugins )
endfunction()

# plugin directories
#add_subdirectory (processes)
#add_subdirectory (ai)
#add_subdirectory (fluidity)
add_subdirectory (barracuda)
