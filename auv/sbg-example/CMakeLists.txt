

include_directories("/usr/local/include/sbgCom/")


# sources
add_executable (
    cube3d EXCLUDE_FROM_ALL


    main.cpp

)

target_link_libraries (
    cube3d
    ${Boost_LIBRARIES}
    SbgComSerial

)
