# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

find_package(ZeroMq REQUIRED)
find_package(Crossroads REQUIRED)
find_package(Ns3 QUIET)

include_directories(${CROSSROADS_INCLUDE_DIR})

add_library(
    zeromq_msg STATIC
    
    zeromq_mailbox.cpp
    zeromq_addresses.cpp
)

target_link_libraries(
    zeromq_msg

    ${CROSSROADS_LIBRARY}
    common
    utility
    messages
    ${rt_LIBRARY}
)


add_executable(
    zmq_test 

    tests/daemon_test.cpp
)

target_link_libraries(
    zmq_test 

    common
    zeromq_msg 
)

if (NS3_FOUND)
    add_executable(
        ns_test

        tests/ns_test.cpp
    )

    target_link_libraries(
        ns_test

        ${NS3_LIBRARIES}
    )
endif()

