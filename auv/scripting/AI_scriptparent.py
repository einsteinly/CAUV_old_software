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
import argparse
from cauv.debug import error, info

from AI_classes import aiScriptOptions, aiScriptState

parser = argparse.ArgumentParser(description = "Script Parent")
parser.add_argument("task")
parser.add_argument("script")
parser.add_argument("--options", "-o", default = {}, type = cPickle.loads)
parser.add_argument("--state", "-s", default = {}, type = cPickle.loads)

args = parser.parse_args()

try:
    script_module = __import__('script_library.' + args.script, fromlist=['script_library'])
except ImportError as e:
    error('Could not import %s from script library' % args.script)
    raise e

#now get the script class, options class and persitant state class
try:
    script_class = script_module.script
except AttributeError as e:
    error('Script file %s does not define a script class' % args.script)
    raise e

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
try:
    script = script_class(args.task, options_class(args.options), state_class(args.state))
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
