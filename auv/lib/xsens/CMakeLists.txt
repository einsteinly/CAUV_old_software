
#set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -D_LOG_TO_STDOUT -D_LOG_CMT1 -D_LOG_CMT2 -D_LOG_CMT3 -D_LOG_CMT_SCAN ")

add_library (
    cmt STATIC

    cmt1.cpp
    cmt2.cpp
    cmt3.cpp
    cmtmessage.cpp
    cmtpacket.cpp
    cmtscan.cpp
    xsens_std.cpp
    xsens_time.cpp
)

target_link_libraries (
    cmt
    ${rt_LIBRARY}
)

