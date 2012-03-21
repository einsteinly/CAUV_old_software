
# sources
add_library (
    model STATIC
    auv_model.cpp
    auv_controller.cpp
)

# libs
target_link_libraries (
    model
    common
    pthread
)
depends_on_messages(model)

add_executable (
    model_test EXCLUDE_FROM_ALL
    model_test.cpp
)

target_link_libraries (
    model_test
    common
    model
)
depends_on_messages(model_test)
