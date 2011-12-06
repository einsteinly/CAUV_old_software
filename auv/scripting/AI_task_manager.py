"""
immediate processing
-stop script requests
queued items
-add/remove conditions
-add/remove tasks
-set task options
-set condition options
periodic - add to queue every x seconds
-check detectors running
-process conditions
-run scripts
"""
import cauv
import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

#todo: check all these ae still neccesary
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

TASK_CHECK_PERIOD = 1
        
class taskManager(aiProcess):
    _store_values = ['task_nid', 'tasks', 'condition_nid', 'conditions', 'detector_nid', 'detectors_required', ]
    #SETUP FUNCTIONS
    def __init__(self, restore):
        aiProcess.__init__(self, 'task_manager')
        #Tasks - list of tasks that (in right conditions) should be called
        self.task_nid = 0
        self.tasks = {}
        #Conditions
        self.condition_nid = 0
        self.conditions = {}
        #queues to add/remove
        self.processing_queue = Queue.Queue()
        self.periodic_timer = None
        #Detectors - definative list of what SHOULD be running
        self.detector_nid = 0
        self.detectors_required = {}
        self.detectors_last_known = Queue.LifoQueue()
        self.detectors_enabled = True
        self.detector_conditions = []
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
            else:
                info('No previous valid state file')
        self.run_processing_loop.set()
    def load_state(self):
        pass
    def save_state(self):
        pass
    
    #ONMESSAGE FUNCTIONS
    def onConditionMessage(self, msg):
        self.processing_queue.put((condition_message, [msg], {}))
    def onTaskMessage(self, msg):
        self.processing_queue.put((process_task, [msg], {}))
    def onAddTaskMessage(self, msg):
        self.processing_queue.put((add_task, [msg.task_type], {}))
    def onRemoveTaskMessage(self, msg):
        self.stop_script()
        self.processing_queue.put((remove_task, [msg.task_id], {}))
    def onTaskOptionsMessage(self, msg):
        self.processing_queue.put((set_task_options, [msg.task_id, msg.task_options])
    @external_function
    def onStopScript(self):
        self.stop_script()
    
    #EXTERNAL FUNCTIONS
    #from detector process
    @external_function
    def on_list_of_detectors(self, detector_list):
        self.detectors_last_known.put(detector_list)
    @external_function
    def on_detector_state_change(self, detector_id, state):
        self.detector_conditions[detector_id].on_state_set(state)
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
                self.tasks[task_id].crash_count += 1
                if self.tasks[task_id].crash_count >= self.tasks[task_id].crash_limit:
                    self.processing_queue.put(('remove_task', [task_id], {}))
                    warning('%s had too many unhandled exceptions, so has been removed from task list.' %(task_id,))
                self.log('Task %s failed after an exception in the script.' %(task_id, ))
            except KeyError:
                warning('Unrecognised task %s crashed (or default script crashed)' %(task_id,))
        elif status == 'SUCCESS':
            self.log('Task %s suceeded, no longer trying to complete this task.' %(task_id, ))
                    self.processing_queue.put(('remove_task', [task_id], {}))
            info('%s has finished succesfully, so is being removed from active tasks.' %(task,))
        else:
            info('%s sent exit message %s' %(task_id, status))
            self.log('Task %s failed, waiting atleast %ds before trying again.' %(task_id, self.tasks[task_id].frequency_limit))
            self.tasks[task_id].last_called = time.time()
        getattr(self.ai,task).confirm_exit()
    #helpful diagnostics
    @external_function
    def export_task_data(self, file_name):
        f = open(file_name, 'w')
        for task in self.tasks.values():
            print task.options
            f.write(task.name+'\n  Options:\n'+'\n'.join(['    '+x[0]+': '+str(x[1]) for x in task.options.items()])+'\n')
        f.close()
            
    #INTERNAL FUNCTIONS
    #add/remove/modify detectors
    def add_detector(self, detector_type, listener):
        detector_id = self.detector_nid
        self.detector_nid += 1
        self.detector_conditions[detector_id] = listener
        self.ai.detector_control.start(detector_id, detector_type)
    def remove_detector(self, detector_id):
        self.detector_conditions.pop(detector_id)
        self.ai.detector_control.stop(detector_id)
    def set_detector_options(self, detector_id, options):
        self.ai.detector_control.set_options(detector_id, options)
    #add/remove/modify tasks
    def add_task(self, task_type):
        task_id = self.task_nid
        self.task_nid += 1
        self.tasks[task_id] = task_type()
        self.tasks[task_id].register(self)
    def remove_task(self, task_id):
        self.tasks[task_id].deregister(self)
        self.tasks.pop(task_id)
    def set_task_options(self, task_id, task_options={}, script_options={}, condition_ids=[]):
        self.tasks[task_id].set_options(options)
        #not only need to change in task, need to try and change in running script
        self.tasks[task_id].set_script_options(options)
        if task_id = self.current_task.id:
            getattr(self.ai, task_id).set_options(options)
        #need to tell task which conditions to use
        #remove current conditions
        for condition in self.tasks[task_id].conditions:
            self.condition.remove(task_id)
        self.tasks[task_id].conditions = []
        #add new conditions
        for condition_id in condition_ids:
            self.tasks[task_id].conditions.append(self.conditions[condition_id])
            self.conditions[condition_id].task_ids.append(task_id)
    #add/remove/modify conditions
    def add_condition(self, condition_type, options):
        condition_id = self.condition_nid
        self.condition_nid += 1
        self.conditions[condition_id] = condition_type(options)
        self.conditions[condition_id].register(self)
    def remove_condition(self, condition_id):
        for task_id in self.conditions[condition_id]:
            self.tasks[task_id].conditions.remove(self.conditions[condition_id])
        self.conditions[condition_id].deregister(self)
        self.conditions.pop(condition_id)
    def set_condition_options(self, condition_id, options):
        self.conditions[condition_id].set_options(options)
    #script control
    def stop_script(self):
        self.ai.auv_control.set_task_id(None)
        #make sure the sub actually stops
        self.ai.auv_control.stop()
        self.ai.auv_control.lights_off()
        if self.running_script:
            try:
                self.running_script.terminate()
            except OSError:
                debug('Could not kill running script (probably already dead)')
    def start_script(self, task_id, script_name, script_opts={}):
        self.ai.auv_control.signal(task_id)
        self.stop_script()
        self.ai.auv_control.set_task_id(task_id)
        info('Starting script: %s  (Task %s)' %(script_name, task_id))
        # Unfortunately if you start a process with ./run.sh (ie in shell) you cant kill it... (kills the shell, not the process)
        self.running_script = subprocess.Popen(['python','./AI_scriptparent.py', task_id, script_name, cPickle.dumps(script_opts)])
    #task control
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
    def process_periodic(self):
        #check tasks
        highest_priority = self.current_priority
        to_start = None
        for task in self.tasks:
            if task != self.current_task and task.is_available() and task.priority > highest_priority and time.time()-task.last_called > task.frequency_limit:
                to_start = task
        if to_start:
            self.start_task(to_start)
        #and finally set a timeout till we want to check again
        self.periodic_timer = threading.Timer(TASK_CHECK_PERIOD, (self.add_periodic_to_queue, [], {}))
    def add_periodic_to_queue(self):
        self.processing_queue.put('process_periodic')
        
    #MAIN LOOP
    def run(self):
        self.periodic_timer = threading.Timer(TASK_CHECK_PERIOD, (self.add_periodic_to_queue, [], {}))
        while True:
            try:
                call = self.processing_queue.get(block=True)
                getattr(self, call[0])(*call[1], **call[2])
            except Error as e:
                error(e.message+' THIS SHOULD REALLY BE FIXED. NOW.'
            self.events.wait(5)
            if self.request_stop.is_set():
                self.request_stop.clear()
                self.stop_script()
    def die():
        try:
            self.periodic_timer.cancel()
        except Exception as error:
            self.debug(error.message)
        super(taskManager, self).die()

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
