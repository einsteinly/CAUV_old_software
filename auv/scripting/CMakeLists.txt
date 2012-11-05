project (scripting)

set (PYMODD ${AUV_SOURCE_DIR}/python-module/)
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target (scripting
    COMMENT "scripting target: depends on python-module"
)
add_dependencies(scripting python-module)

set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
set (RCFILE ${CAUV_SOURCE_DIR}/pylint.rc)
set (PYLINT_M ${CMAKE_CURRENT_BINARY_DIR}/pylint_m)

add_custom_command(
    OUTPUT ${PYLINT_M}
    DEPENDS ${PY_FILES}
    COMMAND touch ${PYLINT_M} && find ${SRCD} -name '*.py' -not -name '*decode_*.py' -print0 | xargs -0 pylint -E --rcfile=${RCFILE} || true
    WORKING_DIRECTORY ${SRCD}    
    COMMENT "running pylint on scripting/*.py"
    SOURCES ${PY_FILES}
)
add_custom_target (
    run-pylint-scripting ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/pylint_m
)

set (MSGD ${SRCD}/../../messages-python/)
set (CHILD ${SRCD}/logging/childecode)
add_custom_target (childecode
    DEPENDS ${MSGS}  ${CHILD}/decode.py.template ${SRCD}/../../.hg
    COMMAND ${MSGD}/msggen/msggen.py -l chil -o ${CHILD} -t ${CHILD} ${MSGS} || true
    WORKING_DIRECTORY ${SRCD}
    COMMENT "generating decode_{HG_CHANGESET}.py for CHIL"
    SOURCES ${MSGS}  ${CHILD}/decode.py.template
)

option(CAUV_USE_VIRTUALENV OFF)
if (${CAUV_USE_VIRTUALENV})
    set(VIRTENVD ${CMAKE_BINARY_DIR}/virtualenv)
    add_custom_command(
        OUTPUT ${VIRTENVD}
        COMMAND virtualenv2 -p python2.7 --no-site-packages --never-download ${VIRTENVD}
        COMMENT "Creating virtualenv"
    )
    add_custom_target (
        create-virtualenv ALL
        DEPENDS ${VIRTENVD}
    )
endif()
