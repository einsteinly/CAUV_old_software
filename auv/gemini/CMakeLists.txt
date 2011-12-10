
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
    ${OpenCV_LIBS}
)
depends_on_messages(gemini_node)
