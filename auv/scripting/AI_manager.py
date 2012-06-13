#!/usr/bin/env python2.7
import subprocess
import time
import argparse
import threading
import utils.watch as watch
import utils.watchfuncs as wf

from cauv.debug import info, debug, warning, error
from cauv import messaging
from cauv.node import Node

from AI_classes import aiProcess, external_function

from utils.ordereddict import OrderedDict

"""
Processes
detector process - runs detector classes, and loops through processing
task manager - listens to detector messages, and manages running scripts
emergency control - runs background checking and can override script in auv_control
scripts - do stuff
auv control - listens to messages from scripts and emergency control

main process keeps sets up things alive
"""
#list of tuples of watch processes and the set of arguments to pass along to them
# i.e. [(process, {'arg_to_pass', 'other arg'}), ...]
processes = [
    (wf.Process('pl_manager',        '{SDIR}',  wf.node_pid('ai_plman'),        wf.restart(3),  None,
                    ['{SDIR}/AI_pipeline_manager.py']),
                        {'disable_gui', 'reset_pls', 'freeze_pls', 'restore'}),
    (wf.Process('auv_control',       '{SDIR}',  wf.node_pid('aiauv_co'),       wf.restart(3),  None, 
                    ['{SDIR}/AI_control_manager.py']),
                         set()),
    (wf.Process('detector_control',  '{SDIR}',  wf.node_pid('aidetect'),  wf.restart(3),  None, 
                    ['{SDIR}/AI_detection_process.py']),
                         {'disable_control'}),
    (wf.Process('task_manager',      '{SDIR}',  wf.node_pid('aitask_m'),  wf.restart(3),  None,
                    ['{SDIR}/AI_task_manager.py']),
                         {'mission', 'restore'}),
    (wf.Process('location',          '{SDIR}',  wf.node_pid('ailocati'),          wf.restart(3),  None,
                    ['{SDIR}/AI_location.py']),
                         set()),
]

class AImanager(aiProcess):
    def __init__(self, opts):
        aiProcess.__init__(self, "manager")
        self.processes = []
        for process, pass_args in processes:
            if process.name in opts['disable']:
                continue
            #'clever' code alert
            process.cmds[0] = process.cmds[0] + ' ' + \
                            ' '.join(('--{}={}'.format(x, opts[x]) if not isinstance(opts[x],bool) else 
                                      ('--' + x if opts[x] else '')
                                      for x in opts if x in pass_args))
            self.processes.append(process)
        self.watcher = watch.Watcher(self.processes, detach=True)
    def _register(self):
        #do this rather than register as registering registers with this process
        print "registering"
        self.node.addObserver(self._msg_observer)
    def run(self):
        self.watcher.monitor(2)
    @external_function
    def register(self, calling_process):
        info('Registering %s' % calling_process)

parser = argparse.ArgumentParser(description="Manage a group of AI processes")
parser.add_argument('--disable', action='append', default=[], help="disable process by name")
p = parser.add_argument_group(title="Arguments passed to subprocesses")
p.add_argument('-r','--restore',      action='store_true',      help="try and resume from last saved state")
p.add_argument('-m','--mission',      default='mission',        help='which mission script to run')
#currently not implemented/not implemented properly
#p.add_argument('--disable-gui',       action='store_true',      help="disable/ignore gui output nodes")
#p.add_argument('--disable-control',   action='store_true',      help="stop AI script from controlling the sub")
#p.add_argument('--reset-pls',         action='store_true',      help="reset pipelines to those stored in /pipelines")
#p.add_argument('--freeze-pls',        action='store_true',      help="ignore changes to the pipeline")
opts = parser.parse_args()
ai = AImanager(opts.__dict__)

try:
    ai.run()
finally:
    ai.watcher.kill(2) #SIGINT, AKA ^C
    ai.node.stop()
