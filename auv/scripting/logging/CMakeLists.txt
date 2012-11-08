#This also needs work
#set (MSGD ${SRCD}/../../messages-python/)
#set (CHILD ${SRCD}/logging/childecode)
#add_custom_target (childecode
#    DEPENDS ${MSGS}  ${CHILD}/decode.py.template ${SRCD}/../../.hg
#    COMMAND ${MSGD}/msggen/msggen.py -l chil -o ${CHILD} -t ${CHILD} ${MSGS} || true
#    WORKING_DIRECTORY ${SRCD}
#    COMMENT "generating decode_{HG_CHANGESET}.py for CHIL"
#    SOURCES ${MSGS}  ${CHILD}/decode.py.template
#)

python_exe(playLog.py)
python_exe(logwatch.py)
python_exe(reIndexLog.py)
python_exe(sonarLogger.py)
install_module(childecode)
install_module(thirdparty) #this should probably be moved to the pip bundle
