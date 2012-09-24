

include_directories("/usr/local/include/sbgCom/")


# sources
add_executable (
    cube3d EXCLUDE_FROM_ALL


    main.cpp

)

find_library(BOOST_THREAD_LIB NAMES boost_thread boost_thread-mt)

target_link_libraries (
    cube3d
    ${BOOST_THREAD_LIB}
    SbgComSerial

)
