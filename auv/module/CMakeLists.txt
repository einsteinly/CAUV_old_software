find_package(ftdi REQUIRED)
include_directories (${ftdi_INCLUDE_DIRS})
link_directories (${ftdi_LIBRARY_DIRS})

option(CAUV_USE_FTDIPP "Whether or not to use the C++ wrapper for FTDI" OFF)
option(CAUV_MCB_IS_FTDI "True if the MCB is an FTDI device" OFF)

if (CAUV_USE_FTDIPP)
    add_definitions(-DCAUV_USE_FTDIPP=1)
endif()
if (CAUV_MCB_IS_FTDI)
    add_definitions(-DCAUV_MCB_IS_FTDI=1)
endif()

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
    ${ftdi_LIBRARIES}
)
