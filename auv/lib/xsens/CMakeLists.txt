find_library (rt_LIBRARY rt)
#message ("rt library is ${rt_LIBRARY}")

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

