# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

add_library (
    cauv_debug STATIC
    
    cauv_debug.cpp
)
target_link_libraries (
    cauv_debug
    
    ${BOOST_THREAD_LIB}
    ${Boost_LIBRARIES}
)
