add_library (
    cauv_debug STATIC
    
    cauv_debug.cpp
)
target_link_libraries (
    cauv_debug
    
    ${BOOST_THREAD_LIB}
    ${Boost_LIBRARIES}
)
