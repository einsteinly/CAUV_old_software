
add_executable(mcb_bridge
    mcb_bridge.cpp
)

target_link_libraries(mcb_bridge
    utility
    cauv_debug

    boost_program_options
    util
)

cauv_install(mcb_bridge)
