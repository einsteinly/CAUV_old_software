
option(CAUV_BUILD_MCB_BRIDGE "True if the MCB bridge shoudl be built" ON)

if(CAUV_BUILD_MCB_BRIDGE)
    add_executable(mcb_bridge
        mcb_bridge.cpp
    )

    target_link_libraries(mcb_bridge
        utility
        cauv_debug

        ${Boost_LIBRARIES}
        util
    )

    cauv_install(mcb_bridge)
endif()
