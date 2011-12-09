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
import argparse
import traceback

from collections import deque

from AI_classes import aiProcess, external_function, RepeatTimer
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
        self.processing_queue = deque()
        self.periodic_timer = RepeatTimer(1, self.add_periodic_to_queue)
        #Detectors - definative list of what SHOULD be running
        self.detector_nid = 0
        self.detectors_last_known = deque()
        self.detectors_enabled = True
        self.detector_conditions = {}
        #Details on whats currently running
        self.running_script = None
        self.current_task = None
        self.current_priority = -1
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
        raise NotImplementedError
    def save_state(self):
        raise NotImplementedError
    
    #ONMESSAGE FUNCTIONS
    def onAddTaskMessage(self, msg):
        self.processing_queue.append(('create_task', [tasks[msg.taskType]], {}))
    def onRemoveTaskMessage(self, msg):
        self.processing_queue.append(('remove_task', [msg.taskId], {}))
    def onSetTaskStateMessage(self, msg):
        self.processing_queue.append(('set_task_options', [msg.taskId, msg.taskOptions, msg.scriptOptions, msg.conditionIds], {}))
        
    def onAddConditionMessage(self, msg):
        self.processing_queue.append(('create_condition', [conditions[msg.conditionType]], {}))
    def onRemoveConditionMessage(self, msg):
        self.processing_queue.append(('remove_condition', [msg.conditionId], {}))
    def onSetConditionStateMessage(self, msg):
        self.processing_queue.append(('set_condition_options', [msg.conditionId, msg.conditionOptions], {}))
    
    def onScriptControlMessage(self, msg):
        if msg.command == messaging.ScriptCommand.Stop:
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
            getattr(self.ai, self.current_task.id).begin_override_pause()
    @external_function
    def notify_end_pause(self, message):
        if self.current_task:
            getattr(self.ai, self.current_task.id).end_override_pause()
    #from script
    @external_function
    def on_script_exit(self, task_id, status):
        self.processing_queue.appendleft(('process_status_message', [task_id, status], {}))
    #helpful diagnostics
    @external_function
    def export_task_data(self, file_name):
        f = open(file_name, 'w')
        for task in self.tasks.values():
            print task.options
            f.write(task.name+'\n  Options:\n'+'\n'.join(['    '+x[0]+': '+str(x[1]) for x in task.options.items()])+'\n')
        f.close()
            
    #INTERNAL FUNCTIONS
    #--functions run on incoming messages--
    #add/remove/modify detectors
    def add_detector(self, detector_type, listener):
        debug("Adding detector of type %s" %str(detector_type))
        detector_id = self.detector_nid
        self.detector_nid += 1
        self.detector_conditions[detector_id] = listener
        self.ai.detector_control.start(detector_id, detector_type)
        return detector_id
    def remove_detector(self, detector_id):
        debug("Detector condition %s" %str(detector_id))
        self.detector_conditions.pop(detector_id)
        self.ai.detector_control.stop(detector_id)
    def set_detector_options(self, detector_id, options):
        debug("Setting options %s on detector %d" %(str(options), detector_id))
        self.ai.detector_control.set_options(detector_id, options)
        
    #add/remove/modify/reg tasks
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
        debug("Removing task %d" %task_id)
        #remove task of given id (don't forget to let the task do any clearing it wants)
        self.tasks[task_id].deregister(self)
        self.tasks.pop(task_id)
        if self.current_task and task_id == self.current_task.id:
            self.stop_script()
        self.node.send(messaging.TaskRemovedMessage(task_id))
    def set_task_options(self, task_id, task_options={}, script_options={}, condition_ids=[]):
        debug("Setting options %s on task %d" %(str((task_options, script_options, condition_ids)), task_id))
        task = self.tasks[task_id]
        task.set_options(task_options)
        #not only need to change in task, need to try and change in running script
        task.set_script_options(script_options)
        if self.current_task and task_id == self.current_task.id:
            getattr(self.ai, str(task_id)).set_options(options)
        #need to tell task which conditions to use
        #remove current conditions
        for condition in task.conditions.itervalues():
            condition.task_ids.remove(task_id)
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
    def create_condition(self, condition_type, options={}):
        debug("Creating condition of type %s" %str(condition_type))
        #create and register with self
        condition = condition_type(options)
        condition.register(self)
        self.node.send(messaging.ConditionStateMessage(condition.id, condition.get_options(), condition.get_debug_values()))
        return condition.id
    def register_condition(self, condition):
        #give conditon an id and add to condition list
        condition_id = self.condition_nid
        self.condition_nid += 1
        self.conditions[condition_id] = condition
        return condition_id
    def remove_condition(self, condition_id):
        debug("Removing condition %d" %condition_id)
        self.conditions[condition_id].deregister(self)
        self.conditions.pop(condition_id)
        self.node.send(messaging.ConditionRemovedMessage(condition_id))
    def set_condition_options(self, condition_id, options):
        debug("Setting options %s on condition %s" %(str(options), condition_id))
        condition = self.conditions[condition_id]
        condition.set_options(options)
        self.node.send(messaging.ConditionStateMessage(condition.id, condition.get_options(), condition.get_debug_values()))
        
    #script control
    def stop_script(self):
        self.ai.auv_control.set_task_id(None)
        #make sure the sub actually stops
        self.ai.auv_control.stop()
        self.ai.auv_control.lights_off()
        if self.running_script:
            try:
                self.running_script.kill()
            except OSError:
                debug('Could not kill running script (probably already dead)')
        info('Stopping Script')
    def process_status_message(self, task_id, status):
        task_id = int(task_id)
        if status == 'ERROR':
            try:
                self.tasks[task_id].options.crash_count += 1
                if self.tasks[task_id].options.crash_count >= self.tasks[task_id].options.crash_limit:
                    self.remove_task(task_id)
                    warning('%d had too many unhandled exceptions, so has been removed from task list.' %(task_id,))
                self.log('Task %d failed after an exception in the script.' %(task_id, ))
            except KeyError:
                warning('Unrecognised task %d crashed (or default script crashed)' %(task_id,))
        elif status == 'SUCCESS':
            self.log('Task %d suceeded, no longer trying to complete this task.' %(task_id, ))
            self.remove_task(task_id)
            info('%d has finished succesfully, so is being removed from active tasks.' %(task_id,))
        else:
            info('%d sent exit message %s' %(task_id, status))
            self.log('Task %d failed, waiting atleast %ds before trying again.' %(task_id, self.tasks[task_id].options.frequency_limit))
        self.tasks[task_id].options.last_called = time.time()
        getattr(self.ai,str(task_id)).confirm_exit()
                
    #--function run by periodic loop--
    def start_script(self, task_id, script_name, script_opts={}):
        self.ai.auv_control.signal(task_id)
        self.stop_script()
        self.ai.auv_control.set_task_id(str(task_id))
        info('Starting script: %s  (Task %s)' %(script_name, task_id))
        # Unfortunately if you start a process with ./run.sh (ie in shell) you cant kill it... (kills the shell, not the process)
        self.running_script = subprocess.Popen(['python2.7','./AI_scriptparent.py', str(task_id), script_name, cPickle.dumps(script_opts)])
    def start_task(self, task):
        #disable/enable detectors according to task
        self.detectors_enabled = task.options.detectors_enabled_while_running
        if self.detectors_enabled: self.ai.detector_control.enable()
        else: self.ai.detector_control.disable()
        #start the new script
        self.start_script(task.id, task.options.script_name, task.get_script_options())
        #set priority
        self.current_priority = task.options.running_priority
        #mark last task not active, current task active
        if self.current_task:
            self.current_task.active = False
        self.current_task = task
        task.active = True
        
    def process_periodic(self):
        #check running script, clear up if has died
        if self.running_script:
            if self.running_script.poll():
                self.running_script = None
                self.current_task = None
                self.current_priority = -1
                #make sure detector are running
        if not self.running_script: #must recheck as set in above if
            if not self.detectors_enabled:
                self.detectors_enabled = True
                self.ai.detector_control.enable()
        #check detectors, sort out anything that has gone wrong here
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
        #check tasks, run as appropriate
        highest_priority = self.current_priority
        to_start = None
        for task in self.tasks.itervalues():
            if task != self.current_task and task.is_available() and \
               task.options.priority > highest_priority and \
               time.time()-task.options.last_called > task.options.frequency_limit:
                to_start = task
                highest_priority = task.options.priority
        if to_start:
            self.start_task(to_start)
    def add_periodic_to_queue(self):
        self.processing_queue.append(('process_periodic', [], {}))
        
    #MAIN LOOP
    def run(self):
        self.periodic_timer.start()
        while True:
            try:
                try:
                    call = self.processing_queue.popleft()
                except IndexError:
                    time.sleep(0.2)
                    continue
                print call
                getattr(self, call[0])(*call[1], **call[2])
            except Exception:
                error(traceback.format_exc())
    def die(self):
        try:
            self.periodic_timer.die = True
        except Exception as error:
            debug(error.message)
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
