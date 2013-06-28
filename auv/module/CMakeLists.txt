# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

# sources
add_library (
    module STATIC
    module.cpp
)

# libs
target_link_libraries (
    module
    common
    ${Boost_LIBRARIES}
)
