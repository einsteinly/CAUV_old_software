# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (heimdall)

find_package(Crossroads REQUIRED)

include_directories(${CROSSROADS_INCLUDE_DIR})

add_executable(heimdall
    heimdall.cpp
)

target_link_libraries(heimdall
    zeromq_msg
    utility

    ${CROSSROADS_LIBRARY}
)
cauv_install(heimdall)
