project (AUV)

option(CAUV_USE_SHITTY_OLD_MCB_SHIT "Use shitty old mcb shit code for the shitty old mcb and not the nice shiny new can shit" OFF)

find_package (OpenCV REQUIRED)
find_package (Eigen REQUIRED)
include_directories (${Eigen_INCLUDE_DIRS})

# we expect the source root dir to be in the include path
include_directories (${AUV_SOURCE_DIR})

# make lib a system path, so that we don't get warnings from it
include_directories (SYSTEM "${AUV_SOURCE_DIR}/lib")

add_subdirectory (camera)
add_subdirectory (control)
add_subdirectory (img-pipeline)
add_subdirectory (lib)
add_subdirectory (python-module)
add_subdirectory (scripting)
add_subdirectory (sonar)
add_subdirectory (gemini)
add_subdirectory (temperature)
if (CAUV_USE_SHITTY_OLD_MCB_SHIT)
    add_subdirectory(mcb_bridge)
endif()
