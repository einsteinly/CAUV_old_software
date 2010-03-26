cmake_minimum_required (VERSION 2.8)

project (CAUV)

# set paths for package search scripts
set (CMAKE_MODULE_PATH ${CAUV_SOURCE_DIR} "${CAUV_SOURCE_DIR}/cmake/")
set (ssrcspread_DIR "${CAUV_SOURCE_DIR}/cmake/")
set (Spread_DIR "${CAUV_SOURCE_DIR}/cmake/")
set (ftdi_DIR "${CAUV_SOURCE_DIR}/cmake/")
set (FTGL_DIR "${CAUV_SOURCE_DIR}/cmake/")
set (CAUVBoost_DIR "${CAUV_SOURCE_DIR}/cmake/")
set (OpenCV_DIR "${CAUV_SOURCE_DIR}/cmake/")

set (
    CMAKE_INCLUDE_PATH
    "${CMAKE_INCLUDE_PATH}"
    "~/install/"
    "/societies/cauv/install"
)

# eugh, hacky
link_directories (
    "~/install/lib/"
    "/societies/cauv/install/lib/"
)

set (ALWAYS_FLAGS "-DPIC -fPIC")
set (DEBUG_FLAGS "-DCAUV_DEBUG -Wall -g")
set (OPTIMISATION_FLAGS "-O3 -ffast-math")

set (CMAKE_CXX_FLAGS "${DEBUG_FLAGS} ${ALWAYS_FLAGS}")
set (CMAKE_CXX_FLAGS_DEBUG "${DEBUG_FLAGS} ${ALWAYS_FLAGS}")
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${DEBUG_FLAGS} ${OPTIMISATION_FLAGS} ${ALWAYS_FLAGS}")
set (CMAKE_CXX_FLAGS_RELEASE "${OPTIMISATION_FLAGS} ${ALWAYS_FLAGS}")
set (CMAKE_CXX_FLAGS_MINSIZEREL "-Os ${ALWAYS_FLAGS}")

set (CMAKE_C_FLAGS "${ALWAYS_FLAGS}")

add_subdirectory (messages-python)
add_subdirectory (auv)
add_subdirectory (gui-pipeline)


