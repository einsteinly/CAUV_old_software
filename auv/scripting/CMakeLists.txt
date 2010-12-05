project (scripting)

set (PYMODD ${PYTHON-MODULE_BUILD_DIR})

find_package(PythonInterp)


add_custom_target (scripting-test
    export PYTHONPATH=$$PYTHONPATH:${PYMODD} && echo $$PYTHONPATH && gdb --args ${PYTHON_EXECUTABLE} ${SRCD}/pycauvtest.py -
    DEPENDS pycauvtest.py
            python-module
    SOURCES pycauvtest.py
    COMMENT "cauv python module test"
)

add_custom_target (scripting
    DEPENDS python-module
    COMMENT "scripting target: depends on python-module"
)

add_custom_target (pycamtest
    export PYTHONPATH=$$PYTHONPATH:${PYMODD} && ${PYTHON_EXECUTABLE} ${SRCD}/camtest.py 
    DEPENDS camtest.py
            python-module
    SOURCES pycauvtest.py
    COMMENT "cauv python module img-pipeline camera test"
)

