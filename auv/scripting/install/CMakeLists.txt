
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

if (${CAUV_USE_VIRTUALENV})
    add_custom_command (
        OUTPUT ${CMAKE_INSTALL_PREFIX}/cauv.pybundle
        COMMAND pip bundle -r ${SRCD}/requirements.txt --download-cache ${CMAKE_CURRENT_BINARY_DIR}/cache ${CMAKE_INSTALL_PREFIX}/cauv.pybundle
        DEPENDS ${SRCD}/requirements.txt
    )
    add_custom_target (
        create-pybundle
        DEPENDS ${CMAKE_INSTALL_PREFIX}/cauv.pybundle
    )
    install(PROGRAMS activate_reloc DESTINATION bin/)
    install(PROGRAMS setup.sh DESTINATION bin/)
endif()
