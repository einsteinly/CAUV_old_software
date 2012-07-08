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
depends_on_messages(heimdall)
