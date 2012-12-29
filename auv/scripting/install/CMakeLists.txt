
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

if (${CAUV_USE_VIRTUALENV})
    if ("${CMAKE_INSTALL_PREFIX}" MATCHES "/usr.*")
        message(FATAL_ERROR "Don't use the virtualenv setup with a system install dir."
                            "Either disable CAUV_USE_VIRTUALENV or change CMAKE_INSTALL_PREFIX")
    endif()
    find_program(VIRTUALENV NAMES virtualenv2 virtualenv)
    if (${VIRTUALENV} STREQUAL "VIRTUALENV-NOTFOUND")
        message(FATAL_ERROR "virtualenv not found")
    endif()
    set(VIRTENVD ${CMAKE_INSTALL_PREFIX})
    add_custom_command(
        OUTPUT ${VIRTENVD}/bin/python2.7
        COMMAND ${VIRTUALENV} -p python2.7 --system-site-packages ${VIRTENVD}
        COMMENT "Creating virtualenv"
    )

    add_custom_command (
        OUTPUT ${VIRTENVD}/cauv.pybundle
        COMMAND pip bundle -r ${SRCD}/requirements.txt --download-cache ${CMAKE_CURRENT_BINARY_DIR}/cache ${CMAKE_INSTALL_PREFIX}/cauv.pybundle
        DEPENDS ${SRCD}/requirements.txt
    )
    add_custom_target (
        create-pybundle
        DEPENDS ${VIRTENVD}/cauv.pybundle
    )
    add_custom_command (
        OUTPUT ${VIRTENVD}/install_marker
        COMMAND . ${VIRTENVD}/bin/activate && 
                pip install -r ${SRCD}/requirements.txt --download-cache ${CMAKE_CURRENT_BINARY_DIR}/cache &&
                touch ${VIRTENVD}/install_marker
        DEPENDS ${SRCD}/requirements.txt
        DEPENDS ${VIRTENVD}/bin/python2.7
    )
    add_custom_target (
        install-requirements ALL
        DEPENDS ${VIRTENVD}/install_marker
    )
    install(PROGRAMS activate_reloc DESTINATION bin/)
    install(PROGRAMS setup.sh DESTINATION bin/)
endif()
