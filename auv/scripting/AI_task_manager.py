import cauv
import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import time
import threading
import subprocess

from AI_classes import aiProcess, external_function

class task():
    def __init__(self, task_name, task_detectors, task_confirmer, task_action, priority=1):
        self.name = task_name
        self.detectors = task_detectors
        self.confirmer = task_confirmer
        self.action = task_action
        self.completed = False
        self.priority = priority

task_list = [
            task('test', ('test_detector',), 'test_confirm', 'test_action'),
            #task('pipe', ('pipe_detect',), 'pipe_confirm', 'pipe_follow'),
            ]
            
#task manager auto generates a list of what it should be running from these 'tasks', basically looking for these tasks and then running appropriate scripts
#TODO move tasks out of task manager file, should also be kept updated eg when task is finished, in case task manager crashes and has to be restarted
            
default_script = 'search'

class taskManager(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'task_manager')
        #the list of tasks which have been setup
        self.setup_tasks = []
        #the list of detectors that should be running etc, this avoids having to recalculate everytime
        self.needed_detectors = set()
        self.detector_mapping = {}
        self.confirmer_mapping = {}
        self.task_lock = threading.Lock()
        self.running_script = None
        self.priority_of_script = 0
        #the running script cannot be changed while this lock is held
        self.running_script_lock = threading.Lock()
        #note we need to lock here to stop any of the detectors were starting taking control before we've finished
        with self.running_script_lock:
            for task in task_list:
                self.setup_task(task)
            #self.start_script(default_script, 0)
    def setup_task(self, task):
        #start detectors
        with self.task_lock:
            for detector in task.detectors:
                self.detector_mapping[detector] = task
                self.ai.detector_control.start(detector)
            self.needed_detectors.update(task.detectors)
            self.confirmer_mapping[task.confirmer] = task
            self.setup_tasks.append(task)
    @external_function
    def update_detectors(self, detector_list):
        with self.task_lock:
            #check against should be running
            missing = self.needed_detectors.difference(set(detector_list))
        for d in missing:
            self.ai.detector_control.start(d)
            debug("restarting detector %s" %(d))
    @external_function
    def on_detection(self, detector):
        """
        it maybe that a detector needs to be able to communicate more than 'i've found something', but for the moment thats all it can do
        """
        #find out what task this should call
        task = self.detector_mapping[detector]
        with self.running_script_lock:
            if self.priority_of_script >= task.priority:
                info("Detector %s found something, but task manager ignored it because the higher or equal priority script %s was already running")
                return
        self.stop_script()
        self.start_script(task.confirmer, task.priority)
    def on_confirmation(self):
        #could possibly do with not relying on the running script being the one that sent the message
        #look up task
        with self.task_lock:
            task = self.confirmer_mapping[self.running_script.name]
        self.stop_script()
        self.start_script(task.action, task.priority)
    @external_function
    def on_script_exit(self, status):
        #TODO do something useful with return status
        self.ai.auv_script.confirm_exit()
    def stop_script(self):
        with self.running_script_lock:
            if self.running_script:
                self.running_script.kill()
    def start_script(self, script_name, priority):
        #if last script is still running, error
        with self.running_script_lock:
            if self.running_script and self.running_script.poll():
                raise Exception("Tried to start new script while previous script still running")
            self.running_script_priority = priority
            self.running_script = subprocess.Popen(['/bin/sh','./run.sh','./script-library/%s.py' %(script_name)])
            self.running_script.name = script_name
    def run(self):
        while True:
            time.sleep(10)
            info("task_manager still alive")
        #sleep

if __name__ == '__main__':
    tm = taskManager()
    tm.run()
