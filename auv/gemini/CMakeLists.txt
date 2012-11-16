
include_directories (${AUV_SOURCE_DIR}/lib/GeminiSDK/Src/SDK)

# sources
add_executable (
    gemini_node
    gemini_node.cpp
)
cauv_install(gemini_node)

# libs
target_link_libraries (
    gemini_node
    common
    GeminiSDK
    pthread
    opencv_image
    ${OpenCV_LIBS}
)


add_executable (
    fake_gemini EXCLUDE_FROM_ALL
    fake_gemini.cpp
)
target_link_libraries (
    fake_gemini
    common
    opencv_image
    ${OpenCV_LIBS}
)

