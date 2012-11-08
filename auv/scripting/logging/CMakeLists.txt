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
python_exe(messageLogger.py)
