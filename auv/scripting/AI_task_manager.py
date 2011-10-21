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
from AI_tasks import tasks
from AI_conditions import conditions

"""          
task manager auto generates a list of what it should be running from these 'tasks', basically looking for these tasks and then running appropriate scripts
"""          

class FakeEvent():
    #more efficient than using an actual event when not neccesary
    def __init__(self, multievent, name):
        self.multievent=multievent
        self.name=name
    def set(self):
        self.multievent.set(name)
    def clear(self):
        self.multievent.clear(name)
    def is_set(self):
        return self.multievent.is_set(name)

class MultiEvent(threading.Event):
    def __init__(self, event_names):
        threading.Event.__init__(self)
        self.event_lock = threading.Lock()
        self.events = dict([(event_name, False) for event_name in event_names])
    def set(self, event_name):
        #set the particular event name, and the general event flag
        if not event_name in self.events:
            raise KeyError('Event %s not part of this MultiEvent' %(event_name,))
        with self.event_lock:
            self.events[event_name] = True
        threading.Event.set(self)
    def clear(self, event_name):
        #clear the particular name, and the general flag if all are cleared
        if not event_name in self.events:
            raise KeyError('Event %s not part of this MultiEvent' %(event_name,))
        with self.event_lock:
            self.events[event_name] = True
            if all(self.events.values()):
                threading.Event.clear(self)
    def get_event(self, event_name):
        if not event_name in self.events:
            raise KeyError('Event %s not part of this MultiEvent' %(event_name,))
        return(FakeEvent(self.events, event_name)
        

class taskManager(aiProcess):
    _store_values = ['task_nid', 'tasks', 'condition_nid', 'conditions', 'detector_nid', 'detectors_required', ]
    #SETUP FUNCTIONS
    def __init__(self, restore):
        aiProcess.__init__(self, 'task_manager')
        self.events = MultiEvent(('conditions','detectors',))
        #Tasks - list of tasks that (in right conditions) should be called
        self.task_nid = 0
        self.tasks = {}
        #queues to add/remove
        self.tasks_to_add = Queue.Queue()
        self.tasks_to_remove = Queue.Queue()
        #Conditions
        self.condition_nid = 0
        self.conditions = {}
        #queues to add/remove
        self.conditions_to_add = Queue.Queue()
        self.conditions_to_remove = Queue.Queue()
        #Detectors - definative list of what SHOULD be running
        self.detector_nid = 0
        self.detectors_required = {}
        self.detectors_last_known = Queue.LifoQueue()
        self.detectors_enabled = True
        #Details on whats currently running
        self.running_script = None
        self.current_task = None
        self.current_priority = 0
        #state data file
        self.state_shelf = shelve.open('state_task_manager.shelf')
        #Setup intial values
        restored = False
        if restore:
            info('Looking for previous states...')
            if self.load_state():
                restored = True
                self.log('Task manager restored.')
            else: info('No previous valid state file')
        self.run_processing_loop.set()
    def load_state(self):
        pass
    def save_state(self):
        pass
    
    #ONMESSAGE FUNCTIONS
    def onConditionMessage(self, msg):
        self.save_state()
    def onTaskMessage(self, msg):
        self.save_state()
    def onRemoveTaskMessage(self, task_ref):
        try:
            with self.task_lock:
                self.task_list[task_ref].deregister(self)
        except KeyError:
            error('Tried to setup non-existant task.')
        if self.task_list[task_ref].active:
            self.request_stop_script()
        self.save_state()
    def onTaskOpts(self, task_ref, opt_dict):
        with self.task_lock:
            self.task_list[task_ref].options.update(opt_dict)
            #try and update options in the running script
            if self.task_list[task_ref].active:
                for x in opt_dict:
                    getattr(self.ai, self.task_list[task_ref].script_name).set_option(x,opt_dict[x])
        self.save_state()
    @external_function
    def onStopScript(self):
        pass
    
    #EXTERNAL FUNCTIONS
    #from detector process
    @external_function
    def on_list_of_detectors(self, detector_list):
        self.detectors_last_known.put(detector_list)
    @external_function
    def on_detector_state_change(self, id, state):
        self.detectors[id].on_state_set(state)
    #from control process
    @external_function
    def notify_begin_pause(self, message):
        if self.current_task:
            getattr(self.ai, self.current_task).begin_override_pause()
    @external_function
    def notify_end_pause(self, message):
        if self.current_task:
            getattr(self.ai, self.current_task).end_override_pause()
    #from script
    @external_function
    def on_script_exit(self, task_id, status):
        if status == 'ERROR':
            try:
                self.task_list[task].crash_count += 1
                if self.task_list[task].crash_count >= self.task_list[task].crash_limit:
                    self.remove_task(task)
                    warning('%s had too many unhandled exceptions, so has being removed from the active tasks.' %(task,))
                self.log('Task %s failed after an exception in the script.' %(task, ))
            except KeyError:
                warning('Unrecognised task %s crashed (or default script crashed)' %(task,))
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
    def export_task_data(self, file_name):
        with self.task_lock:
            f = open(file_name, 'w')
            for task in self.task_list.values():
                print task.options
                f.write(task.name+'\n  Options:\n'+'\n'.join(['    '+x[0]+': '+str(x[1]) for x in task.options.items()])+'\n')
            f.close()
            
    #INTERNAL FUNCTIONS
    def add_detector(self, detector_name, listener):
        id = self.detector_nid
        self.detector_nid += 1
        self.detectors[id] = [listener]
        self.ai.detector_control.start(id, detector_name)
    def remove_detector(self, id):
        self.detectors.pop(id)
        self.ai.detector_control.stop(id)
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
        
    #MAIN LOOP
    def run(self):
        #processing list
        #-stop script requests
        #-add/remove conditions
        #-add/remove tasks
        #-set task options
        #-set condition options
        #-check detectors running
        #-process conditions
        #-run scripts (always)
        while True:
            self.events.wait(5)
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
                        to_start = self.task_list[self.req_start_task]
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

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-r', '--restore', dest='restore', default=False,
                 action='store_true', help="try and resume from last saved state")
    opts, args = p.parse_args()
    
    tm = taskManager(**opts.__dict__)
    try:
        tm.run()
    finally:
        tm.die()
