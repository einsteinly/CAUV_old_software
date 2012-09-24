add_library (
    cauv_debug STATIC
    
    cauv_debug.cpp
)
find_library(BOOST_THREAD_LIB boost_thread boost_thread-mt)
target_link_libraries (
    cauv_debug
    
    ${BOOST_THREAD_LIB}
    boost_date_time
    boost_program_options
    boost_system
    boost_filesystem
)
