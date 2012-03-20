

include_directories("/usr/local/include/sbgCom/")


# sources
add_executable (
    cube3d EXCLUDE_FROM_ALL


    main.c

)

target_link_libraries (
    cube3d

    SbgComSerial

)
