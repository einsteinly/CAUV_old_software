# required libraries:
find_package (OpenCV REQUIRED)

option(CAUV_ZEROMQ_MESSAGING "use zeromq for messaging instead of spread" OFF)
option(CAUV_ZMQ3 "use zmq3 for zeromq" OFF)
if(CAUV_ZEROMQ_MESSAGING)
    if(CAUV_ZMQ3)
        add_subdirectory(zeromq3)
    else()
        add_subdirectory(zeromq)
    endif()
    add_definitions(-DZEROMQ_MESSAGING)
else()
    add_subdirectory(spread)
endif()

add_subdirectory(supermess)

set (
    common_files

    cauv_global.cpp
    cauv_node.cpp
    cauv_utils.cpp
    image.cpp
    math.cpp
    #random.cpp
)

set (
    common_links

    messages
    cauv_debug
    ${OpenCV_LIBS}
    boost_thread
    boost_date_time
    boost_iostreams
    boost_signals
    boost_program_options
)

if (ZEROMQ_MESSAGING)
    list(APPEND common_links zeromq_msg)
else()
    list(APPEND common_links spread_msg)
endif()

add_library (
    common STATIC

    ${common_files}
)

target_link_libraries (
    common

    ${common_links}
)
depends_on_messages(common)

# dynamic lib:
add_library (
    common-shared SHARED
    
    ${common_files}
)
target_link_libraries (
    common-shared

    ${common_links}
)
depends_on_messages(common-shared)
cauv_install(common-shared)

add_dependencies(common logo-headers version-header)
add_dependencies(common-shared logo-headers version-header)

