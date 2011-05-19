import cauv
import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import time
import threading
import subprocess
import cPickle

from os.path import getmtime

from AI_classes import aiProcess, external_function

from mission import task_list, default_script
"""          
task manager auto generates a list of what it should be running from these 'tasks', basically looking for these tasks and then running appropriate scripts
TODO:
-should also have a current status file, in case task manager crashes and has to be restarted
-record when a task is finished... this could be done either using the on_script_exit return value or remove_task called directly from the script
"""          

class taskManager(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'task_manager')
        #These variables maybe accessesed by more than one thread
        #Tasks - list of tasks that (in right conditions) should be called
        self.task_lock = threading.RLock()
        self.active_tasks = []
        #Conditions - note although conditions may be changed externally, the list of conditions may not
        self.conditions = {}
        self.conditions_changed = threading.Event()
        #Detectors - definative list of what SHOULD be running
        self.detector_lock = threading.Lock()
        self.active_detectors = {}
        #These should only be accessed by this thread - details on whats currently running
        self.running_script = None
        self.current_task = None
        self.current_priority = 0
        #Setup intial values
        try:
            info('Looking for previous states...')
            self.load_state()
        except:
            info('No previous valid state file, loading from mission.py')
            #Open state file (must be done before changing the state)
            self.state_file = open('status.tmp', 'r+')
            with self.task_lock:
                for task in task_list:
                    task.register(self)
        #Force evaluation of tasks
        self.conditions_changed.set()
    def add_detector(self, detector_name, listener):
        with self.detector_lock:
            if not (detector_name in self.active_detectors):
                self.active_detectors[detector_name] = []
                self.ai.detector_control.start(detector_name)
            self.active_detectors[detector_name].append(listener)
        self.save_state()
    def remove_detector(self, detector_name, listener):
        with self.detector_lock():
            self.active_detectors[detector_name].remove(listener)
            if not self.active_detectors[detector_name]:
                self.active_detectors.pop(detector_name)
        self.save_state()
    def load_state(self):
        #TODO actually load a state (see save state)
        #self.active_tasks=
        #self.active_detectors=
        #self.conditions=
        #if getmtime('mission.py') != state.mtime:
        #    error('Mission.py appears to have been modified. This may cause error when restoring the previous state.')
        raise Exception
    def save_state(self):
        #TODO work out way of storing state
        pass
        #self.state_file.truncate()
        #with self.task_lock, self.detector_lock:
        #    state.mtime = getmtime('mission.py')
        #    state.save()
        #self.state_file.flush()
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
        self.save_state()
    @external_function
    def notify_detector(self, detector_name, *args, **kwargs):
        for listener in self.active_detectors[detector_name]:
            listener.set_state(*args, **kwargs)
        self.conditions_changed.set()
        #Note, no saving state for detectors, as they are dynamic so there state shouldn't be saved
    @external_function
    def add_task(self, task_ref):
        #TODO
        #need to be able to match tasks in mission.py to actual tasks
        #should then add to the lists and do basic setup (watch locked data)
        #could call save_state, but check threading (save state not ext_func)
        pass
    @external_function
    def remove_task(self, task_ref):
        #TODO see add_task, note need to be able to remove task based on task ref. Again need to watch potential threading issues
        pass
    @external_function
    def on_script_exit(self, status):
        #TODO do something useful with return status
        getattr(self.ai,self.current_task).confirm_exit()
        #Force immediate recheck
        self.conditions_changed.set()
    def stop_script(self):
        if self.running_script:
            try:
                self.running_script.terminate()
            except OSError:
                debug('Could not kill running script (probably already dead)')
        #make sure the sub actually stops
        self.ai.control_manager.stop()
        #TODO tell control to stop listening to the script
    def start_script(self, script_name):
        self.stop_script()
        info('Starting script: '+script_name)
        # Unfortunately if you start a process with ./run.sh (ie in shell) you cant kill it... (kills the shell, not the process)
        self.running_script = subprocess.Popen(['python','./AI_scriptparent.py', script_name, script_name])
    def start_default_script(self):
        self.current_priority = 0
        self.current_task = None
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
            if not self.running_script:
                debug('No script was running, returning to default')
                self.start_default_script()
            if self.running_script.poll() != None:
                error('The running script appears to have stopped, returning to default.')
                self.start_default_script()
            info("task_manager still alive")
        #sleep

if __name__ == '__main__':
    tm = taskManager()
    tm.run()
