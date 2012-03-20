

include_directories("/usr/local/include/sbgCom/")


# sources
add_executable (
    cube3d


    main.c

)

target_link_libraries (
    cube3d

    SbgComSerial

)
