find_package(sensors REQUIRED)

include_directories (SYSTEM "${sensors_INCLUDE_DIR}")

# sources
add_executable (
    temperature EXCLUDE_FROM_ALL

    TempNode.cpp
)
cauv_install ( temperature )

# libs
target_link_libraries (
    temperature
    
    ${sensors_LIBRARIES}
    common
)

