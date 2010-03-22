# required libraries:
find_package (ssrcspread)
include_directories (${ssrcspread_INCLUDE_DIRS} REQUIRED) 

# abbreviate a little:
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

if (USE_C_BASED_MSGGEN) 
    # c-based message generation 
    # use makefiles to build msg-generator
    add_custom_command (
       OUTPUT ${MSGGEN_SOURCE_DIR}/bin/msg-generator
       WORKING_DIRECTORY ${MSGGEN_SOURCE_DIR}
       COMMAND make
    )

    # how to generate messages.cpp and messages.h
    add_custom_command (
       OUTPUT ${SRCD}/messages.cpp ${SRCD}/messages.h
       COMMAND ${MSGGEN_SOURCE_DIR}/bin/msg-generator -o ${SRCD}/messages ${SRCD}/messages.msg 
       DEPENDS ${MSGGEN_SOURCE_DIR}/bin/msg-generator ${SRCD}/messages.msg
       COMMENT "Generating messages.cpp and mesages.h with messages (old c++)" 
    )
else (USE_C_BASED_MSGGEN) 
    # python-based message generation
    # how to generate messages.cpp and messages.h
    set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
    add_custom_command (
       OUTPUT ${SRCD}/messages.cpp
              ${SRCD}/messages.h
       WORKING_DIRECTORY ${PYMSGD}
       COMMAND python msggen.py -o ${SRCD}/messages ${SRCD}/messages.msg 
       DEPENDS ${PYMSGD}/msggen.py
               ${PYMSGD}/message.template.cpp
               ${PYMSGD}/message.template.h
               ${PYMSGD}/msggenyacc.py
               ${PYMSGD}/msggenlex.py
               ${SRCD}/messages.msg
       COMMENT "Generating messages.cpp and mesages.h with messages-python" 
    ) 
endif (USE_C_BASED_MSGGEN)

# rule to make cauv_logo_large.h
add_custom_command (
    OUTPUT ${SRCD}/cauv_logo_large.h
    WORKING_DIRECTORY ${SRCD}
    COMMAND xxd -i - cauv_logo_large.h < cauv_logo_large.txt
    DEPENDS ${SRCD}/cauv_logo_large.txt
    COMMENT "Generating cauv_logo_large.h with xxd"
)

# sources:
add_library (
    common STATIC
    
    cauv_logo_large.h
    cauv_global.cpp
    cauv_node.cpp
    cauv_utils.cpp
    messages.cpp
    spread/cauv_mailbox_monitor.cpp
    spread/cauv_spread_mailbox.cpp
    spread/cauv_spread_messages.cpp
)

# libs:
target_link_libraries (
    common

    ${ssrcspread_LIBRARIES}
    ${OpenCV_LIBS}
    boost_thread
    boost_serialization
    boost_date_time
    boost_iostreams
)


