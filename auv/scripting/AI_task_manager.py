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
import Queue
import subprocess
import cPickle
import shelve
import argparse
import traceback

from collections import deque

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
        self.detectors_last_known = deque()
        self.detectors_enabled = True
        self.detector_conditions = {}
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
    def load_state(self):
        pass
    def save_state(self):
        pass
    
    #ONMESSAGE FUNCTIONS
    def onAddTaskMessage(self, msg):
        self.processing_queue.put(('create_task', [tasks[msg.taskType]], {}))
    def onRemoveTaskMessage(self, msg):
        self.stop_script()
        self.processing_queue.put(('remove_task', [msg.taskId], {}))
    def onSetTaskStateMessage(self, msg):
        self.processing_queue.put(('set_task_options', [msg.taskId, msg.taskOptions, msg.scriptOptions, msg.conditionIds], {}))
        
    def onAddConditionMessage(self, msg):
        self.processing_queue.put(('create_condition', [conditions[msg.conditionType]], {}))
    def onRemoveConditionMessage(self, msg):
        self.processing_queue.put(('remove_condition', [msg.conditionId], {}))
    def onSetConditionStateMessage(self, msg):
        self.processing_queue.put(('set_condition_options', [msg.conditionId, msg.conditionOptions], {}))
    
    @external_function
    def onStopScript(self):
        self.stop_script()
    
    #EXTERNAL FUNCTIONS
    #from detector process
    @external_function
    def on_list_of_detectors(self, detector_list):
        self.detectors_last_known.append(detector_list)
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
        debug("Adding detector of type %s" %str(detector_type))
        detector_id = self.detector_nid
        self.detector_nid += 1
        self.detector_conditions[detector_id] = listener
        self.ai.detector_control.start(detector_id, detector_type)
        return detector_id
    def remove_detector(self, detector_id):
        debug("Removing condition %s" %str(condition_id))
        self.detector_conditions.pop(detector_id)
        self.ai.detector_control.stop(detector_id)
    def set_detector_options(self, detector_id, options):
        debug("Setting options %s on detector %s" %(str(options),str(detector_id)))
        self.ai.detector_control.set_options(detector_id, options)
        
    #add/remove/modify/reg/dereg tasks
    def create_task(self, task_type):
        #create task of named type
        debug("Creating task of type %s" %str(task_type))
        task = task_type()
        task.register(self)
        self.node.send(messaging.TaskStateMessage(task.id,
                task.conditions.keys(),
                task.get_options(),
                task.get_dynamic_options(),
                task.get_static_options(),
                task.active))
        return task.id
    def register_task(self, task):
        #give the task an id
        task_id = self.task_nid
        self.task_nid += 1
        self.tasks[task_id] = task
        return task_id
    def remove_task(self, task_id):
        debug("Removing task %s" %str(task_id))
        #remove task of given id (don't forget to let the task do any clearing it wants)
        self.tasks[task_id].deregister(self)
        self.tasks.pop(task_id)
        self.node.send(messaging.TaskRemovedMessage(task_id))
    def set_task_options(self, task_id, task_options={}, script_options={}, condition_ids=[]):
        debug("Setting options %s on task %s" %(str((task_options, script_options, condition_ids)),str(task_id)))
        task = self.tasks[task_id]
        task.set_options(task_options)
        #not only need to change in task, need to try and change in running script
        task.set_script_options(script_options)
        if self.current_task and task_id == self.current_task.id:
            getattr(self.ai, task_id).set_options(options)
        #need to tell task which conditions to use
        #remove current conditions
        for condition in task.conditions.itervalues():
            condition.remove(task_id)
        task.conditions = {}
        #add new conditions
        for condition_id in condition_ids:
            task.conditions[condition_id] = self.conditions[condition_id]
            self.conditions[condition_id].task_ids.append(task_id)
        self.node.send(messaging.TaskStateMessage(task.id,
                task.conditions.keys(),
                task.get_options(),
                task.get_dynamic_options(),
                task.get_static_options(),
                task.active))
            
    #add/remove/modify conditions
    def create_condition(self, condition_type, options):
        debug("Creating condition of type %s" %str(condition_type))
        #create and register with self
        condition = condition_type(options)
        condition.register(self)
        return condition.id
    def register_condition(self, condition):
        #give conditon an id and add to condition list
        condition_id = self.condition_nid
        self.condition_nid += 1
        self.conditions[condition_id] = condition
        return condition_id
    def remove_condition(self, condition_id):
        debug("Removing condition %s" %str(condition_id))
        self.conditions[condition_id].deregister(self)
        self.conditions.pop(condition_id)
    def set_condition_options(self, condition_id, options):
        debug("Setting options %s on condition %s" %(str(options),str(condition_id)))
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
        #check detectors
        try:
            running_detectors = self.detectors_last_known.pop()
        except IndexError:
            pass
        else:
            self.detectors_last_known.clear()
            for d_id, listener in self.detector_conditions.iteritems():
                if not d_id in running_detectors:
                    self.ai.detector_control.start(d_id, listener.detector_name)
                    self.set_detector_options(d_id, listener.options.get_options())
        #check tasks
        highest_priority = self.current_priority
        to_start = None
        for task in self.tasks.itervalues():
            if task != self.current_task and task.is_available() and task.priority > highest_priority and time.time()-task.last_called > task.frequency_limit:
                to_start = task
        if to_start:
            self.start_task(to_start)
        #and finally set a timeout till we want to check again
        self.periodic_timer = threading.Timer(TASK_CHECK_PERIOD, self.add_periodic_to_queue)
        self.periodic_timer.start()
    def add_periodic_to_queue(self):
        self.processing_queue.put(('process_periodic', [], {}))
        
    #MAIN LOOP
    def run(self):
        self.periodic_timer = threading.Timer(TASK_CHECK_PERIOD, self.add_periodic_to_queue)
        self.periodic_timer.start()
        while True:
            try:
                call = self.processing_queue.get(block=True)
                getattr(self, call[0])(*call[1], **call[2])
            except Exception:
                error(traceback.format_exc())
    def die(self):
        try:
            self.periodic_timer.cancel()
        except Exception as error:
            self.debug(error.message)
        super(taskManager, self).die()

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-r', '--restore', dest='restore', default=False,
                 action='store_true', help="try and resume from last saved state")
    opts, args = p.parse_known_args()
    
    tm = taskManager(**opts.__dict__)
    try:
        tm.run()
    finally:
        tm.die()
