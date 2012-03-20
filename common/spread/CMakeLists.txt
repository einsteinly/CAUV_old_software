find_package (ssrcspread REQUIRED)
include_directories (${ssrcspread_INCLUDE_DIR})
link_directories (${ssrcspread_LIBRARY_DIR})

add_library(
    spread_msg

    mailbox_monitor.cpp
    spread_mailbox.cpp
    spread_rc_mailbox.cpp
    spread_messages.cpp
)
depends_on_messages(spread_msg)

target_link_libraries(spread_msg
    ${ssrcspread_LIBRARIES}
)
