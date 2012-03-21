#!/usr/bin/env python2.7
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

from AI_classes import aiScriptOptions, aiScriptState

if __name__ == '__main__':
    try:
        #first check we were passed required parameters
        try:
            task_id = sys.argv[1]
            script_name = sys.argv[2]
        except IndexError as e:
            error('Tried to run script with wrong command parameters: '+str(sys.argv))
            raise e
        #next see if any arguments or persistent state was passed
        try:
            script_opts = cPickle.loads(sys.argv[3])
        except Exception as e:
            info('No valid options set for script, using default')
            script_opts = {}
        try:
            state = cPickle.loads(sys.argv[4])
        except Exception as e:
            info('No valid persistent state found, using default')
            state = {}
        #try to import the script module
        try:
            script_module = __import__('script_library.'+script_name,fromlist=['script_library'])
        except ImportError as e:
            error('Could not import '+script_name+' from script library')
            raise e
        #now get the script class, options class and persitant state class
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
        try:
            state_class = script_module.scriptState
        except AttributeError:
            info('No default state found for script, assuming none')
            state_class = aiScriptState
        #create class instaces, and run
        script = script_class(task_id, options_class(script_opts), state_class(state))
        result = script.run()
        #notify of result
        script._notify_exit(result)
    except Exception as e:
        #pass error back to task manager
        error(traceback.format_exc())
        script._notify_exit('ERROR')
        raise e
    finally:
        try:
            script.die()
        except NameError:
            error("Could not clear up after the script since it wasn't created properly")
