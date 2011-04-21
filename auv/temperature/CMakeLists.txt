find_package(sensors)

if (sensors_FOUND)

    include_directories (SYSTEM "${sensors_INCLUDE_DIR}")

    # sources
    add_executable (
        temperature

        TempNode.cpp
    )
    cauv_install ( temperature )

    # libs
    target_link_libraries (
        temperature
        
        ${sensors_LIBRARIES}
        common
    )

endif()
