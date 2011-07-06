import cauv
import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import time
import threading
import subprocess
import cPickle
import shelve
import optparse
import traceback

from os.path import getmtime

from AI_classes import aiProcess, external_function

"""          
task manager auto generates a list of what it should be running from these 'tasks', basically looking for these tasks and then running appropriate scripts
"""          

class taskManager(aiProcess):
    def __init__(self, mission, restore):
        aiProcess.__init__(self, 'task_manager')
        #These variables maybe accessesed by more than one thread
        #Tasks - list of tasks that (in right conditions) should be called
        self.task_lock = threading.RLock()
        self.active_tasks = []
        self.task_list = {}
        self.request_stop = threading.Event()
        self.req_start_task = None
        #Conditions - note although conditions may be changed externally, the list of conditions may not
        self.conditions = {}
        self.conditions_changed = threading.Event()
        #Detectors - definative list of what SHOULD be running
        self.detector_lock = threading.Lock()
        self.active_detectors = {}
        self.detectors_enabled = True
        #These should only be accessed by this thread - details on whats currently running
        self.running_script = None
        self.current_task = None
        self.current_priority = 0
        #state data file
        self.state = shelve.open(mission+'_state.shelf')
        #Setup intial values
        restored = False
        if restore:
	    info('Looking for previous states...')
            if self.load_state():
                restored = True
                self.log('Task manager restored.')
            else: info('No previous valid state file, loading from mission.py')
        if not restored:    
            self.mission = __import__(mission)
            self.task_list = {}
            with self.task_lock:
                for task in self.mission.task_list:
                    if task.name in self.task_list:
                        warning('More than one task has the same name, the last task in the list will overwrite earlier tasks')
                    self.task_list[task.name] = task
                for task in self.mission.initial_tasks:
                    self.task_list[task].register(self)
            self.new_state(mission)
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
        with self.detector_lock:
            self.active_detectors[detector_name].remove(listener)
            if not self.active_detectors[detector_name]:
                self.active_detectors.pop(detector_name)
                self.ai.detector_control.stop(detector_name)
        self.save_state()
    def load_state(self):
        if 'state_set' in self.state:
            if getmtime(self.state['mission_name']+'.py') != self.state['mtime']:
                warning('The mission script may have been modified. Continuing, but could cause errors')
            self.mission = __import__(self.state['mission_name'])
            with self.task_lock:
                self.task_list = self.state['task_list']
                for task in self.task_list.values():
                    if task.registered:
                        task.registered = False #oteherwise it will think its already setup -> error
                        task.register(self)
                    #TODO remove a task if it causes repeated crashes?
            return True
        #TODO Make sure to clear up any mess if this fails (ie reset all variables etc)
        return False
    @external_function
    def save_state(self):
        with self.task_lock:
            self.state['task_list'] = self.task_list
        self.state.sync()
    def new_state(self, mission_name):
        #some extra stuff to recorded the 1st time a state is set
        self.state['mission_name'] = mission_name
        self.state['mtime'] = getmtime(mission_name+'.py')
        self.state['state_set'] = True
        self.save_state()
    @external_function
    def update_detectors(self, detector_list):
        with self.detector_lock:
            if not self.detectors_enabled:
                #should have been disabled, try again
                self.ai.detector_control.disable()
                return
            #check against should be running
            missing = set(self.active_detectors.keys())-set(detector_list)
            additional  = set(detector_list)-set(self.active_detectors.keys())
        for d in missing:
            self.ai.detector_control.start(d)
            debug("restarting detector %s" %(d))
        for d in additional:
            self.ai.detector_control.stop(d)
            debug("stopping detector %s" %(d))
    @external_function
    def notify_condition(self, condition_name, *args, **kwargs):
        self.conditions[condition_name].set_state(*args, **kwargs)
        self.log('Condition '+condition_name+' sent parameters: '+', '.join([', '.join(map(str, args)),', '.join(['='.join(map(str,kwarg)) for kwarg in kwargs])]))
        self.conditions_changed.set()
        self.save_state()
    @external_function
    def notify_detector(self, detector_name, *args, **kwargs):
        self.log('Detector '+detector_name+' sent parameters: '+', '.join([', '.join(map(str,args)),', '.join(['='.join(map(str,kwarg)) for kwarg in kwargs])]))
        for listener in self.active_detectors[detector_name]:
            listener.set_state(*args, **kwargs)
        self.conditions_changed.set()
        #Note, no saving state for detectors, as they are dynamic so there state shouldn't be saved
    @external_function
    def add_task(self, task_ref):
        try:
            with self.task_lock:
                self.task_list[task_ref].register(self)
        except KeyError:
            error('Tried to setup non-existant task.')
        self.save_state()
    @external_function
    def remove_task(self, task_ref):
        try:
            with self.task_lock:
                self.task_list[task_ref].deregister(self)
        except KeyError:
            error('Tried to setup non-existant task.')
        if self.task_list[task_ref].active:
            self.request_stop_script()
        self.save_state()
    @external_function
    def modify_task_options(self, task_ref, opt_dict):
        with self.task_lock:
            self.task_list[task_ref].options.update(opt_dict)
            #try and update options in the running script
            if self.task_list[task_ref].active:
                for x in opt_dict:
                    getattr(self.ai, self.task_list[task_ref].script_name).set_option(x,opt_dict[x])
        self.save_state()
    @external_function
    def export_task_data(self, file_name):
        with self.task_lock:
            f = open(file_name, 'w')
            for task in self.task_list.values():
                print task.options
                f.write(task.name+'\n  Options:\n'+'\n'.join(['    '+x[0]+': '+str(x[1]) for x in task.options.items()])+'\n')
            f.close()
    @external_function
    def on_script_exit(self, task, status):
        if status == 'ERROR':
            self.task_list[task].crash_count += 1
            if self.task_list[task].crash_count >= self.task_list[task].crash_limit:
                self.remove_task(task)
                warning('%s had too many unhandled exceptions, so has being removed from the active tasks.' %(task,))
            self.log('Task %s failed after an exception in the script.' %(task, ))
        elif status == 'SUCCESS':
            self.log('Task %s suceeded, no longer trying to complete this task.' %(task, ))
            self.remove_task(task)
            info('%s has finished succesfully, so is being removed from active tasks.' %(task,))
        else:
            info('%s sent exit message %s' %(task, status))
            self.log('Task %s failed, waiting atleast %ds before trying again.' %(task, self.task_list[task].frequency_limit))
            self.task_list[task].last_called = time.time()
        getattr(self.ai,task).confirm_exit()
        #Force immediate recheck
        time.sleep(0.5) # otherwise script won't have a chance to stop
        self.conditions_changed.set()
    @external_function
    def request_stop_script(self):
        self.request_stop.set()
        self.conditions_changed.set()
    @external_function
    def request_start_task(self, task_ref):
        self.req_start_task = task_ref
        self.conditions_changed.set()
    def stop_script(self):
        if self.running_script:
            try:
                self.running_script.terminate()
            except OSError:
                debug('Could not kill running script (probably already dead)')
        #make sure the sub actually stops
        self.ai.pipeline_manager.drop_script_pls()
        self.ai.auv_control.stop()
        self.ai.auv_control.lights_off()
        self.ai.auv_control.set_task_id(None)
    def start_script(self, task_ref, script_name, script_opts={}):
        self.ai.auv_control.signal(task_ref)
        self.stop_script()
        self.ai.auv_control.set_task_id(task_ref)
        info('Starting script: %s  (Task %s)' %(script_name, task_ref))
        # Unfortunately if you start a process with ./run.sh (ie in shell) you cant kill it... (kills the shell, not the process)
        self.running_script = subprocess.Popen(['python','./AI_scriptparent.py', task_ref, script_name, cPickle.dumps(script_opts)])
    def start_default_script(self):
        self.current_priority = 0
        self.current_task = None
        self.start_script('default', self.mission.default_script, self.mission.default_script_options)
        self.log('Starting the default script, since no tasks are currently available to complete.')
        #also make sure detectors are running
        with self.detector_lock:
            self.detectors_enabled = True
        self.ai.detector_control.enable()
    def start_task(self, task):
        #disable/enable detectors according to task
        with self.detector_lock:
            self.detectors_enabled = task.detectors_enabled
            if self.detectors_enabled: self.ai.detector_control.enable()
            else: self.ai.detector_control.disable()
        #start the new script
        self.start_script(task.name, task.script_name, task.options)
        #set priority
        self.current_priority = task.running_priority
        #mark last task not active, current task active
        if self.current_task:
            self.task_list[self.current_task].active = False
        self.current_task = task.name
        task.active = True
    def run(self):
        while True:
            self.conditions_changed.wait(5)
            if self.conditions_changed.is_set():
                self.conditions_changed.clear()
                if self.request_stop.is_set():
                    self.request_stop.clear()
                    self.stop_script()
                with self.task_lock:
                    highest_priority = self.current_priority
                    to_start = None
                    for task in self.active_tasks:
                        if task.script_name != self.current_task and task.priority > highest_priority and time.time()-task.last_called > task.frequency_limit:
                            if task.is_available():
                                to_start = task
                    if self.req_start_task:
                        try:
                            to_start = aelf.task_list[self.req_start_task]
                        except KeyError:
                            error("cannot start %s, not an active task." %(self.req_start_task,))
                        self.req_start_task = None
                    if to_start:
                        self.start_task(to_start)
            if not self.running_script:
                debug('No script was running, returning to default')
                self.start_default_script()
            if self.running_script.poll() != None:
                error('The running script appears to have stopped, returning to default.')
                self.start_default_script()
            info("task_manager still alive")
        #sleep

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-r', '--restore', dest='restore', default=False,
                 action='store_true', help="try and resume from last saved state")
    p.add_option('-m', '--mission', dest='mission', default='mission',
                 type=str, action='store', help='which mission script to run (default = mission)')
    opts, args = p.parse_args()
    
    tm = taskManager(**opts.__dict__)
    try:
        tm.run()
    finally:
        tm.die()
