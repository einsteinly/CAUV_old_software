project (AUV)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${AUV_BINARY_DIR}/bin")

# everything needs boost: 
find_package (Boost 1.40 REQUIRED)
set (LIBS ${LIBS} ${Boost_LIBRARIES}) 
include_directories (SYSTEM ${Boost_INCLUDE_DIRS})
link_directories (${Boost_LIBRARY_DIRS})
message ("-- Boost include dirs: ${Boost_INCLUDE_DIRS}")
message ("-- Boost lib dirs: ${Boost_LIBRARY_DIRS}")

# openCV paths:
find_package (OpenCV REQUIRED)
include_directories (SYSTEM ${OpenCV_INCLUDE_DIRS})
message ("-- OpenCV include dir: ${OpenCV_INCLUDE_DIRS}")
message ("-- OpenCV libs: ${OpenCV_LIBS}")


# we expect the source root dir to be in the include path
include_directories (${AUV_SOURCE_DIR})

# make lib a system path, so that we don't get warnings from it
include_directories (SYSTEM "${AUV_SOURCE_DIR}/lib")


# subdirectories:
add_subdirectory (camera)
add_subdirectory (common)
add_subdirectory (control)
add_subdirectory (fileinput)
add_subdirectory (img-pipeline)
add_subdirectory (img-viewer)
add_subdirectory (lib)
add_subdirectory (module)
add_subdirectory (pinger)
add_subdirectory (webcam)
