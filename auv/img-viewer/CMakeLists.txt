
# sources
add_executable (
    img-viewer
    img-viewer.cpp
)

# libs
target_link_libraries (
    img-viewer
    common
    cv
    highgui
)


