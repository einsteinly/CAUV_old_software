project (scripting)

set (PYMODD ${AUV_SOURCE_DIR}/python-module/)
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

find_package(PythonInterp)


#add_custom_target (scripting-test
#    export PYTHONPATH=$$PYTHONPATH:${PYMODD} && echo $$PYTHONPATH && gdb --args ${SRCD}/run.sh ${SRCD}/pycauvtest.py -
#    DEPENDS pycauvtest.py
#    SOURCES pycauvtest.py
#    COMMENT "cauv python module test"
#)
#add_dependencies(scripting-test python-module)


add_custom_target (scripting
    COMMENT "scripting target: depends on python-module"
)
add_dependencies(scripting python-module)

#message ("Python Module Build Dir: ${PYMODD}")

#add_custom_target (pycamtest
#    export PYTHONPATH=$$PYTHONPATH:${PYMODD} && echo $$PYTHONPATH && ${SRCD}/run.sh ${SRCD}/camtest.py 
#    DEPENDS camtest.py
#    SOURCES pycauvtest.py
#    COMMENT "cauv python module img-pipeline camera test"
#)
#add_dependencies(pycamtest python-module)


set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
set (RCFILE ${CAUV_SOURCE_DIR}/pylint.rc)
file (GLOB PY_FILES "${SRCD}/*.py")
file (GLOB EXTRA_PY_FILES "${SRCD}/*/*.py")
add_custom_target(
    run-pylint-scripting ALL
    DEPENDS ${PY_FILES}
    COMMAND pylint -E --rcfile=${RCFILE} ${PY_FILES} ${EXTRA_PY_FILES} || true
    WORKING_DIRECTORY ${SRCD}    
    COMMENT "running pylint on scripting/*.py"
    SOURCES ${PY_FILES}
)

