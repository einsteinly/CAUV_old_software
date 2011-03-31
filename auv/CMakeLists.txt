project (AUV)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${AUV_BINARY_DIR}/bin")

# openCV paths:
find_package (OpenCV REQUIRED)

# we expect the source root dir to be in the include path
include_directories (${AUV_SOURCE_DIR})

# make lib a system path, so that we don't get warnings from it
include_directories (SYSTEM "${AUV_SOURCE_DIR}/lib")

option(CAUV_NO_TEMPERATURE "True if temperature node should not be built" OFF)

add_subdirectory (camera)
add_subdirectory (control)
add_subdirectory (controlv2)
add_subdirectory (fileinput)
add_subdirectory (img-pipeline)
add_subdirectory (img-viewer)
add_subdirectory (lib)
add_subdirectory (module)
add_subdirectory (pervert)
add_subdirectory (pinger)
add_subdirectory (python-module)
add_subdirectory (scripting)
add_subdirectory (send)
add_subdirectory (sonar)
add_subdirectory (webcam)
add_subdirectory (model)
if (NOT CAUV_NO_TEMPERATURE)
    add_subdirectory (temperature)
endif()
