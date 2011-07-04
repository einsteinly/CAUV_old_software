"""
run command: /bin/sh ./run.sh AI_scriptparent.py TASK_NAME SCRIPT_NAME
where SCRIPT_NAME is the name of the script in the script_library folder
and TASK_NAME is the name of the task, will respond to ai.TASK_NAME.func()
This basically runs AI_scripts, as they are implemented as classes, so this creates and instance and calls the run method.
"""
import sys
import cPickle
import traceback
from cauv.debug import error, info

from AI_classes import aiScriptOptions, aiProcess

if __name__ == '__main__':
    try:
        try:
            task_ref = sys.argv[1]
            script_name = sys.argv[2]
        except IndexError as e:
            error('Tried to run script with wrong command parameters: '+str(sys.argv))
            raise e
        try:
            script_opts = cPickle.loads(sys.argv[3])
        except Exception as e:
            info('No valid options set for script, using default')
            script_opts = {}
        try:
            script_module = __import__('script_library.'+script_name,fromlist=['script_library'])
        except ImportError as e:
            error('Could not import '+script_name+' from script library')
            raise e
        try:
            script_class = script_module.script
        except AttributeError:
            error('Script file '+script_name+' does not define a script class')
            raise Exception
        try:
            options_class = script_module.scriptOptions
        except AttributeError:
            info('No default options found for script, assuming none')
            options_class = aiScriptOptions
        script = script_class(task_ref, options_class(script_opts))
        script.run()
    except Exception as e:
        ainode = aiProcess('script_error_reporter')
        ainode.ai.auv_control.stop()
        ainode.ai.task_manager.on_script_exit(task_ref, 'ERROR')
        error(traceback.format_exc())
        ainode.die()
        raise e
    finally:
        script.die()
