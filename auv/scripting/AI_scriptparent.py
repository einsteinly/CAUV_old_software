"""
run command: /bin/sh ./run.sh AI_scriptparent.py SCRIPT_NAME SCRIPT_AI_REFRENCE
where SCRIPT_NAME is the name of the script in the script_library folder
and SCRIPT_AI_REFERENCE is the name that it will respond to ie ai.SCRIPT_AI_REFERENCE.func()
This basically runs AI_scripts, as they are implemented as classes, so this creates and instance and calls the run method.
"""
import sys
from cauv.debug import error

if __name__ == '__main__':
    try:
        script_name = sys.argv[1]
        script_ref = sys.argv[2]
    except IndexError as e:
        error('Tried to run script with wrong command parameters: '+str(sys.argv))
        raise e
    try:
        script_module = __import__('script_library.'+script_name,fromlist=['script_library'])
    except ImportError as e:
        error('Could not import '+script_name+' from script library')
        raise e
    try:
        script = script_module.script(script_name)
    except AttributeError:
        error('Script file '+script_name+' does not define a script class')
    script.run()
