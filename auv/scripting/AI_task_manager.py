import cauv
import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import time
import threading
import subprocess

from AI_classes import aiProcess, detectorCondition, aiTask, external_function

task_list = [
            aiTask('test', 1, conditions = [detectorCondition('test_detector','test'),]),
            #task('pipe', ('pipe_detect',), 'pipe_confirm', 'pipe_follow'),
            ]
"""          
task manager auto generates a list of what it should be running from these 'tasks', basically looking for these tasks and then running appropriate scripts
TODO: -move tasks out of task manager file,
-should also have a current status file, in case task manager crashes and has to be restarted
-record when a task is finished...
"""          
default_script = 'spiral'

class taskManager(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'task_manager')
        self.task_lock = threading.RLock()
        #the list of tasks which have been setup
        self.active_tasks = []
        self.conditions = {}
        self.conditions_changed = threading.Event()
        #the list of detectors that should be running etc, this avoids having to recalculate everytime
        self.detector_lock = threading.Lock()
        self.active_detectors = {}
        self.running_script_lock = threading.RLock()
        self.running_script = None
        self.current_task = None
        self.current_priority = 0
        #the running script cannot be changed while this lock is held
        #note we need to lock here to stop any of the detectors were starting taking control before we've finished
        with self.running_script_lock:
            for task in task_list:
                task.register(self)
            #self.start_script(default_script, 0)
    def add_detector(self, detector_name, listener):
        with self.detector_lock:
            if not (detector_name in self.active_detectors):
                self.active_detectors[detector_name] = []
                self.ai.detector_control.start(detector_name)
            self.active_detectors[detector_name].append(listener)
    def remove_detector(detector_name, listener):
        with self.detector_lock():
            self.active_detectors[detector_name].remove(listener)
            if not self.active_detectors[detector_name]:
                self.active_detectors.pop(detector_name)
    @external_function
    def update_detectors(self, detector_list):
        with self.detector_lock:
            #check against should be running
            missing = set(self.active_detectors.keys())-set(detector_list)
        for d in missing:
            self.ai.detector_control.start(d)
            debug("restarting detector %s" %(d))
    @external_function
    def notify_condition(self, condition_name, *args, **kwargs):
        self.conditions[condition_name].set_state(*args, **kwargs)
        self.conditions_changed.set()
    @external_function
    def notify_detector(self, detector_name, *args, **kwargs):
        for listener in self.active_detectors[detector_name]:
            listener.set_state(*args, **kwargs)
        self.conditions_changed.set()
    @external_function
    def on_script_exit(self, status):
        #TODO do something useful with return status
        getattr(self.ai,self.current_task).confirm_exit()
    def stop_script(self):
        #TODO make sure the sub actually stops...
        with self.running_script_lock:
            if self.running_script:
                try:
                    self.running_script.terminate()
                except OSError:
                    debug('Could not kill running script (probably already dead)')
    def start_script(self, script_name):
        with self.running_script_lock:
            self.stop_script()
            info('Starting script: '+script_name)
            # Unfortunately if you start a process with ./run.sh (ie in shell) you cant kill it... (kills the shell, not the process)
            self.running_script = subprocess.Popen(['python','./AI_scriptparent.py', script_name, script_name])
    def start_default_script(self):
        with self.task_lock:
            self.current_priority = 0
            self.current_task = None
            #note still inside the lock as we dont want to accidentally let task_manager start some other task, then us reset the script to default
            self.start_script(default_script)
    def run(self):
        while True:
            if self.conditions_changed.wait(5):
                self.conditions_changed.clear()
                with self.task_lock:
                    for task in self.active_tasks:
                        if task.script_name != self.current_task and task.priority > self.current_priority:
                            if task.is_available():
                                self.start_script(task.script_name)
                                self.current_priority = task.running_priority
                                self.current_task = task.script_name
            with self.running_script_lock:
                if not self.running_script:
                    debug('No script was running, returning to default')
                    self.start_default_script()
                if self.running_script.poll() != None:
                    error('The running script appears to have stopped, returning to default.')
                    self.start_default_script()
            #info("task_manager still alive")
        #sleep

if __name__ == '__main__':
    tm = taskManager()
    tm.run()
