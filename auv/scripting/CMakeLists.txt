project (scripting)

set (PYMODD ${AUV_SOURCE_DIR}/python-module/)
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target (scripting
    COMMENT "scripting target: depends on python-module"
)
add_dependencies(scripting python-module)

#TODO: fix this up
#set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
#set (RCFILE ${CAUV_SOURCE_DIR}/pylint.rc)
#set (PYLINT_M ${CMAKE_CURRENT_BINARY_DIR}/pylint_m)
#
#add_custom_command(
#    OUTPUT ${PYLINT_M}
#    DEPENDS ${PY_FILES}
#    COMMAND touch ${PYLINT_M} && find ${SRCD} -name '*.py' -not -name '*decode_*.py' -print0 | xargs -0 pylint -E --rcfile=${RCFILE} || true
#    WORKING_DIRECTORY ${SRCD}    
#    COMMENT "running pylint on scripting/*.py"
#    SOURCES ${PY_FILES}
#)
#add_custom_target (
#    run-pylint-scripting ALL
#    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/pylint_m
#)

function(install_module MODULE)
    install(DIRECTORY ${MODULE} DESTINATION ${CAUV_PYTHON_INSTALL_PATH}
            FILES_MATCHING PATTERN "*.py")
endfunction()

function(python_exe EXE)
    install(PROGRAMS ${EXE} DESTINATION bin/)
endfunction()

function(config_dir DIRECTORY)
    install(DIRECTORY ${DIRECTORY} DESTINATION config/)
endfunction()

install_module(cauv)
install_module(utils)
install_module(simulator)
install_module(gamepad_maps)
install_module(AI)
install_module(detector_library)
install_module(script_library)

python_exe(gamepad_server.py)
python_exe(ishell.py)
python_exe(last-resort.py)
python_exe(module-test.py)
python_exe(motortest.py)
python_exe(penultimate-resort.py)
python_exe(persist.py)
python_exe(pingThatShit.py)
python_exe(run.sh)
python_exe(savepl.py)
python_exe(sim.py)
python_exe(telemetryLogger.py)
python_exe(temperaturemonitor.py)
python_exe(wasd_remote.py)
python_exe(watch.py)
python_exe(watchctl.py)

install(PROGRAMS startup/setup_cameras.sh DESTINATION bin/)

config_dir(sessions)
config_dir(missions)
config_dir(pipelines)
config_dir(persist)

add_subdirectory(logging)
add_subdirectory(AI)
add_subdirectory(CAN)
add_subdirectory(heimdall)

add_subdirectory(install) #should be last
