project (AUV)

# TODO: set bin directory somehow?

# set paths for package search scripts
set (CMAKE_MODULE_PATH ${AUV_SOURCE_DIR} "${AUV_SOURCE_DIR}/cmake/")
set (ssrcspread_DIR "${AUV_SOURCE_DIR}/cmake/")
set (Spread_DIR "${AUV_SOURCE_DIR}/cmake/")
set (ftdi_DIR "${AUV_SOURCE_DIR}/cmake/")


# everything needs boost: 
find_package (Boost 1.40)

if (Boost_VERSION LESS 104000)
    message(WARNING "Boost version too low (${Boost_VERSION}), assuming SRCF paths")
    set (Boost_INCLUDE_DIRS "/societies/cauv/install/include/")
    set (Boost_LIBRARY_DIRS "/societies/cauv/install/lib/")
endif (Boost_VERSION LESS 104000)

set (LIBS ${LIBS} ${Boost_LIBRARIES}) 
include_directories (${Boost_INCLUDE_DIRS})
link_directories (${Boost_LIBRARY_DIRS})
message ("Boost include dirs: ${Boost_INCLUDE_DIRS}")
message ("Boost lib dirs: ${Boost_LIBRARY_DIRS}")

# openCV paths:
find_package (OpenCV)
include_directories (${OpenCV_INCLUDE_DIRS})
message ("OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message ("OpenCV libs: ${OpenCV_LIBS}")


# we expect the source root dir to be in the include path
include_directories (${AUV_SOURCE_DIR})


# subdirectories:
add_subdirectory (camera)
add_subdirectory (common)
#dd_subdirectory (control)
add_subdirectory (fileinput)
add_subdirectory (img-pipeline)
add_subdirectory (img-viewer)
add_subdirectory (lib)
add_subdirectory (module)
add_subdirectory (pinger)
add_subdirectory (webcam)

