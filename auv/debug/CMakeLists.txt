add_library (
    cauv_debug STATIC
    
    cauv_debug.cpp
)
target_link_libraries (
    cauv_debug

    boost_thread
    boost_date_time
    boost_program_options
    boost_system
)

