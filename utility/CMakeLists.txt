# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (utility)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${AUV_BINARY_DIR}/bin")

include_directories (${AUV_SOURCE_DIR})

add_library(utility

    math.cpp
    string.cpp
    uid.cpp
    time.cpp
    ratelimit.cpp
    random.cpp
    coordinates.cpp
    daemon.cpp
    files.cpp
)
add_precompiled_header(utility ${CAUV_SOURCE_DIR}/pch.h)

if(CAUV_BUILD_TESTS)
    add_executable(
        serialisation-test
        serialisation-test.cpp
    )
    target_link_libraries(
       serialisation-test
       cauv_debug
    )

    add_executable(
        async-test
        async-test.cpp
    )
    target_link_libraries(
        async-test
        cauv_debug
        pthread
    )

endif()
