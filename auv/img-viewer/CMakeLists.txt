
# sources
add_executable (
    img-viewer EXCLUDE_FROM_ALL
    img-viewer.cpp
)

# libs
target_link_libraries (
    img-viewer
    common
    ${OpenCV_LIBS}
)


