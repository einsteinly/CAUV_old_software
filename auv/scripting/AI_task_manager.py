#!/usr/bin/env python2
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

import time
import subprocess
import cPickle
import shelve
import argparse
import traceback

from collections import deque

from AI_classes import aiProcess, external_function, RepeatTimer
from AI_tasks import tasks as task_classes
from AI_conditions import conditions as condition_classes

from utils.conv import BoostMapToDict
import utils.event as event

"""          
task manager auto generates a list of what it should be running from these 'tasks',
basically looking for these tasks and then running appropriate scripts
"""          

TASK_CHECK_INTERVAL = 1
STATE_SAVE_INTERVAL = 30

class taskManager(aiProcess):
    #_store_values = ['task_nid', 'tasks', 'condition_nid', 'conditions', 'detector_nid', 'detectors_required', ]
    #SETUP FUNCTIONS
    def __init__(self, opts):
        aiProcess.__init__(self, 'task_manager')
        self.node.subMessage(messaging.AddTaskMessage())
        self.node.subMessage(messaging.RemoveTaskMessage())
        self.node.subMessage(messaging.SetTaskStateMessage())
        self.node.subMessage(messaging.AddConditionMessage())
        self.node.subMessage(messaging.RemoveConditionMessage())
        self.node.subMessage(messaging.SetConditionStateMessage())
        self.node.subMessage(messaging.RequestAIStateMessage())
        self.node.subMessage(messaging.ScriptControlMessage())
        #start paused
        self.all_paused = True
        #Tasks - list of tasks that (in right conditions) should be called
        self.task_nid = 0
        self.tasks = {}
        #Conditions
        self.condition_nid = 0
        self.conditions = {}
        #Detectors - definitive list of what SHOULD be running
        self.detectors_last_known = deque()
        self.detectors_enabled = True
        self.detector_conditions = {}
        #Details on whats currently running
        self.running_script = None
        self.current_task = None
        self.current_priority = -1
        self.additional_tasks = {} #task_id: (task, script)
        #dict of data known by task manager used by conditions
        self.tm_info = {'latitude':None,
                        'longitude':None,
                        'depth':None,}
        #state data file
        self.state_shelf = shelve.open(opts.mission)
        #Setup intial values
        info('Looking for previous states...')
        if self.load_state():
            self.log('Task manager restored.')
            info('Found and loaded previous state.')
        else:
            info('No previous valid state file')
        self.gui_send_all()
        
        #start receiving messages
        self._register()

    def load_state(self, include_persist=False):
        #check whether there is a state
        if (not 'tasks' in self.state_shelf) or (not 'conditions' in self.state_shelf):
            return False
        old2new_ids = {}
        #load and set conditions first, so that they can be linked to tasks
        for condition_id, (condition_type_name, condition_options) in self.state_shelf['conditions'].iteritems():
            try:
                condition_type = condition_classes[condition_type_name]
            except KeyError:
                warning("Condition type %s no longer exists, could cause problems" %condition_type_name)
                continue
            old2new_ids[condition_id] = self.create_condition(condition_type)
            self.set_condition_options(old2new_ids[condition_id], condition_options)
        for task_id, (task_type_name, task_options, task_script_options, condition_ids, persist_state) in self.state_shelf['tasks'].iteritems():
            try:
                task_type = task_classes[task_type_name]
            except KeyError:
                warning("Task type %s no longer exists, could cause problems" %task_type_name)
                continue
            task_id = self.create_task(task_type, default_conditions=False)
            self.set_task_options(task_id, task_options, task_script_options, [old2new_ids[condition_id] for condition_id in condition_ids])
            if include_persist:
                self.tasks[task_id].persist_state = persist_state
        return True

    @event.repeat_event(STATE_SAVE_INTERVAL, True)
    def save_state(self):
        debug('Saving mission state.')
        task_dict = {}
        cond_dict = {}
        for condition_id, condition in self.conditions.iteritems():
            cond_dict[condition_id] = (condition.__class__.__name__, condition.get_options())
        for task_id, task in self.tasks.iteritems():
            task_dict[task_id] = (task.__class__.__name__, task.get_options(), 
                    task.get_script_options(), task.get_condition_ids(), task.persist_state)
        self.state_shelf['tasks'] = task_dict
        self.state_shelf['conditions'] = cond_dict
        self.state_shelf.sync()
    
    #ONMESSAGE FUNCTIONS
    @event.event_func
    def onAddTaskMessage(self, msg):
        self.create_task(task_classes[msg.taskType])
    @event.event_func
    def onRemoveTaskMessage(self, msg):
        self.remove_task(msg.taskId)
    @event.event_func
    def onSetTaskStateMessage(self, msg):
        self.set_task_options(msg.taskId, BoostMapToDict(msg.taskOptions),
                              BoostMapToDict(msg.scriptOptions),
                              msg.conditionIds)
        
    @event.event_func
    def onAddConditionMessage(self, msg):
        self.create_condition(condition_classes[msg.conditionType])
    @event.event_func
    def onRemoveConditionMessage(self, msg):
        self.remove_condition(msg.conditionId)
    @event.event_func
    def onSetConditionStateMessage(self, msg):
        self.set_condition_options(msg.conditionId, BoostMapToDict(msg.conditionOptions))
    @event.event_func
    def onScriptControlMessage(self, msg):
        command = msg.command.name
        task_id = msg.taskId
        if command in ('Stop','Restart'):
            if task_id == self.current_task.id:
                #stop main script
                self.stop_current_script()
            elif task_id in self.additional_tasks:
                self.stop_script(task_id)
            else:
                warning('Tried to remove non-existant task %s' %(task_id))
        if command == 'Restart':
            self.tasks[task_id].persist_state = {}
        if command in ('Start', 'Restart'):
            try:
                task = self.tasks[task_id]
            except KeyError:
                warning('Tried to start non-existant task %s' %(task_id))
            self.start_script(task)
        elif command == 'Pause':
            if task_id in self.tasks:
                self.tasks[task_id].paused = True
                if self.tasks[task_id].active:
                    self.ai.auv_control.pause_script(task_id)
        elif command == 'Resume':
            if task_id in self.tasks:
                self.tasks[task_id].paused = False
                if self.tasks[task_id].active:
                    self.ai.auv_control.resume_script(task_id)
        elif command == 'PauseAll':
            self.all_paused = True
            self.ai.auv_control.pause()
        elif command == 'ResumeAll':
            debug('Resumed')
            self.all_paused = False
            self.ai.auv_control.resume()

    @event.event_func
    def onRequestAIStateMessage(self, msg):
        self.gui_send_all()
    
    #MESSAGES TO GUI
    def gui_update_task(self, task):
        self.node.send(messaging.TaskStateMessage(task.id,
                task.conditions.keys(),
                task.get_options(),
                task.get_dynamic_options_as_params(),
                task.get_static_options_as_params(),
                task.active))
    def gui_update_condition(self, condition):
        if not condition._suppress_reporting: #eg detector conditions
            self.node.send(messaging.ConditionStateMessage(condition.id,
                condition.get_options(), condition.get_debug_values(), []))
    def gui_remove_task(self, task_id):
        self.node.send(messaging.TaskRemovedMessage(task_id))
    def gui_remove_condition(self, condition_id):
        self.node.send(messaging.ConditionRemovedMessage(condition_id))

    def gui_send_all(self):
        #send type info
        self.node.send(messaging.TaskTypesMessage(list(task_classes.keys())))
        self.node.send(messaging.ConditionTypesMessage(
            {condition_name: condition.get_pipeline_names() for 
                condition_name, condition in condition_classes.iteritems()}))

        #send conditions (first, since tasks depend on conditions)
        for condition in self.conditions.itervalues():
            self.gui_update_condition(condition)
        #send tasks
        for task in self.tasks.itervalues():
            self.gui_update_task(task)
    
    #EXTERNAL FUNCTIONS
    #from detector process
    @external_function
    def on_list_of_detectors(self, detector_list):
        self.detectors_last_known.append(detector_list)
    @external_function
    @event.event_func
    def on_detector_state_change(self, detector_id, state):
        self.detector_conditions[detector_id].on_state_set(state)
    #from script
    @external_function
    @event.event_priority(-1)
    @event.event_func
    def on_script_exit(self, task_id, status):
        if status == 'ERROR':
            try:
                self.tasks[task_id].options.crash_count += 1
                if self.tasks[task_id].options.crash_count >= self.tasks[task_id].options.crash_limit:
                    self.remove_task(task_id)
                    warning('%s had too many unhandled exceptions, so has been removed from task list.' %(task_id,))
                self.log('Task %s failed after an exception in the script.' %(task_id, ))
            except KeyError:
                warning('Unrecognised task %s crashed' %(task_id,))
            self.tasks[task_id].options.last_called = time.time()
        elif status == 'SUCCESS':
            self.log('Task %s suceeded, no longer trying to complete this task.' %(task_id, ))
            self.remove_task(task_id)
            info('%s has finished succesfully, so is being removed from active tasks.' %(task_id,))
        else:
            info('%s sent exit message %s' %(task_id, status))
            self.log('Task %s failed, waiting atleast %ds before trying again.' %(task_id, self.tasks[task_id].options.frequency_limit))
            self.tasks[task_id].options.last_called = time.time()
        getattr(self.ai,str(task_id)).confirm_exit()

    @external_function
    @event.event_func
    def modify_script_options(self, task_id, options):
        self.set_task_options(task_id, options)

    @external_function
    @event.event_func
    def on_persist_state_change(self, task_id, key, attr):
        self.tasks[task_id].persist_state[key] = attr

    #from location process
    @external_function
    @event.event_func
    def broadcast_position(self, position):
        self.tm_info['latitude'] = position.latitude
        self.tm_info['longitude'] = position.longitude
        self.tm_info['depth'] = -position.altitude
        for task_id in self.additional_tasks:
            getattr(self.ai, task_id)._set_position(position)
        if self.current_task:
            getattr(self.ai, self.current_task.id)._set_position(position)

    #helpful diagnostics
    @external_function
    def export_task_data(self, file_name):
        f = open(file_name, 'w')
        for task in self.tasks.values():
            print task.options
            f.write(task.name+'\n  Options:\n'+'\n'.join(['    '+x[0]+': '+str(x[1]) 
                                                            for x in task.options.items()])+'\n')
        f.close()
            
    #INTERNAL FUNCTIONS
    #--functions run on incoming messages--
    #add/remove/modify detectors
    def add_detector(self, detector_type, listener):
        debug("Adding detector of type %s" %str(detector_type))
        self.detector_conditions[listener.id] = listener
        self.ai.detector_control.start(listener.id, detector_type)
    def remove_detector(self, detector_id):
        debug("Detector condition %s" %detector_id)
        self.detector_conditions.pop(detector_id)
        self.ai.detector_control.stop(detector_id)
    def set_detector_options(self, detector_id, options):
        debug("Setting options %s on detector %s" %(str(options), detector_id))
        self.ai.detector_control.set_options(detector_id, options)
        
    #add/remove/modify/reg tasks
    def create_task(self, task_type, default_conditions=True):
        #create task of named type
        debug("Creating task of type %s" %str(task_type))
        task = task_type()
        task.register(self)
        if default_conditions:
            task.add_default_conditions(self)
        #self.gui_update_task(task) skip here since is already sent by updating task options
        return task.id

    def register_task(self, task):
        #give the task an id
        task_id = task.__class__.__name__+str(self.task_nid)
        self.task_nid += 1
        self.tasks[task_id] = task
        return task_id

    def remove_task(self, task_id):
        debug("Removing task %s" %task_id)
        #remove task of given id (don't forget to let the task do any clearing it wants)
        self.tasks[task_id].deregister(self)
        self.tasks.pop(task_id)
        if self.current_task and task_id == self.current_task.id:
            self.stop_current_script()
        elif task_id in self.additional_tasks:
            self.stop_script(task_id)
        self.gui_remove_task(task_id)

    def set_task_options(self, task_id, task_options={}, script_options={}, condition_ids=[]):
        debug("Setting options %s on task %s" %(str((task_options, script_options, condition_ids)), task_id))
        task = self.tasks[task_id]
        task.set_options(task_options)
        #not only need to change in task, need to try and change in running script
        task.set_script_options(script_options)
        if (self.current_task and task_id == self.current_task.id) or task_id in self.additional_tasks:
            getattr(self.ai, str(task_id)).set_options(script_options)
        #need to tell task which conditions to use
        #remove current conditions
        for condition in task.conditions.itervalues():
            condition.task_ids.remove(task_id)
        task.conditions = {}
        #add new conditions
        for condition_id in condition_ids:
            task.conditions[condition_id] = self.conditions[condition_id]
            self.conditions[condition_id].task_ids.append(task_id)
        self.gui_update_task(task)

    #add/remove/modify conditions
    def create_condition(self, condition_type, options={}):
        debug("Creating condition of type %s" %str(condition_type))
        #create and register with self
        #work out if uses tm_info
        if 'tm_info' in condition_type.__init__.im_func.func_code.co_varnames:
            condition = condition_type(options, self.tm_info)
        else:
            condition = condition_type(options)
        condition.register(self)
        self.gui_update_condition(condition)
        return condition.id
    def register_condition(self, condition):
        #give conditon an id and add to condition list
        condition_id = condition.__class__.__name__+str(self.condition_nid)
        self.condition_nid += 1
        self.conditions[condition_id] = condition
        return condition_id
    def remove_condition(self, condition_id):
        debug("Removing condition %s" %condition_id)
        self.conditions[condition_id].deregister(self)
        self.conditions.pop(condition_id)
        self.gui_remove_condition(condition_id)
    def set_condition_options(self, condition_id, options):
        debug("Setting options %s on condition %s" %(str(options), condition_id))
        condition = self.conditions[condition_id]
        condition.set_options(options)
        self.gui_update_condition(condition)
        
    #script control
    def stop_script(self, task_id):
        #make sure additional task isnt blocking other tasks from doing stuff, and doesn't have any leftover pipelines
        self.ai.auv_control.remove_additional_task_id(task_id)
        self.ai.pl_manager.drop_task_pls(task_id)
        script = self.additional_tasks.pop(task_id)[1]
        if script:
            try:
                script.kill()
            except OSError:
                debug('Could not kill running script (probably already dead)')
        info('Stopping additional script for task %s' %task_id)

    def stop_current_script(self):
        self.ai.auv_control.set_current_task_id(None, 0)
        self.ai.pl_manager.drop_task_pls(self.current_task.id)
        if self.running_script:
            try:
                self.running_script.kill()
            except OSError:
                debug('Could not kill running script (probably already dead)')
        info('Stopping Script')
        
    def stop_all_scripts(self):
        for task_id in self.additional_tasks.keys():
            self.stop_script(task_id)
        self.stop_current_script()

    def start_script(self, task):
        if self.all_paused or task.paused:
            return
        #start the new script
        self.ai.auv_control.signal(task.id)
        info('Starting script: %s  (Task %s)' %(task.options.script_name, task.id))
        # Unfortunately if you start a process with ./run.sh (ie in shell) you cant kill it... (kills the shell, not the process)
        script = subprocess.Popen(['python2.7','./AI_scriptparent.py', str(task.id),
                                   task.options.script_name, 
                                   '--options',
                                   cPickle.dumps(task.get_script_options()), 
                                   '--state',
                                   cPickle.dumps(task.persist_state)])
        if task.options.solo:
            if self.current_task:
                #mark last task not active, current task active
                self.current_task.active = False
                self.stop_current_script()
            self.running_script = script
            self.ai.auv_control.set_current_task_id(task.id, task.options.priority)
            #disable/enable detectors according to task
            self.detectors_enabled = task.options.detectors_enabled_while_running
            if self.detectors_enabled:
                self.ai.detector_control.enable()
            else:
                self.ai.detector_control.disable()
            #set priority
            self.current_priority = task.options.running_priority
            self.current_task = task
            self.current_task.active = True
        else:
            #make sure an instance is not already running
            if task.id in self.additional_tasks:
                self.stop_script(task.id)
                warning("Detected script already started, killing old script first")
            #then just add it to the list
            #THIS MIGHT BREAK (RAPID REMOVAL/ADDITION OF THE SAME ID)
            self.ai.auv_control.add_additional_task_id(task.id, task.options.running_priority)
            self.additional_tasks[task.id]=(task,script)
        task.active = True
        #update task status to gui
        self.node.send(messaging.TaskStateMessage(task.id,
                task.conditions.keys(),
                task.get_options(),
                task.get_dynamic_options(),
                task.get_static_options(),
                task.active))
        
    #--function run by periodic loop--
    @event.repeat_event(TASK_CHECK_INTERVAL, True)
    def process_periodic(self):
        #check running script, clear up if has died
        if self.running_script:
            if self.running_script.poll():
                self.running_script = None
                self.current_task = None
                self.current_priority = -1
        dead_scripts = []
        for task_id, (task, script) in self.additional_tasks.iteritems():
            if script.poll():
                dead_scripts.append(task_id)
        for task_id in dead_scripts:
            self.additional_tasks.pop(task_id)
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
            if task.is_available():
                if (not task.options.solo):
                    if not task.id in self.additional_tasks:
                        self.start_script(task)
                elif task != self.current_task and \
                    task.options.priority > highest_priority and \
                    time.time()-task.options.last_called > task.options.frequency_limit:
                    to_start = task
                    highest_priority = task.options.priority
        if to_start:
            self.start_script(to_start)
        #rebroadcast condition info (since might have changin debug vals etc)
        for condition in self.conditions.itervalues():
            self.gui_update_condition(condition)
        
    def die(self):
        try:
            #kill any child scripts (since not managed by AI_manager, so may get left around)
            self.stop_all_scripts()
        except Exception as error:
            debug(error.message)
        aiProcess.die(self)

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-r', '--restore', dest='restore', default=False,
                 action='store_true', help="try and resume from last saved state")
    p.add_argument('-m', '--mission', dest='mission', default='mission',
                 action='store', help="try and resume from last saved state")
    opts, args = p.parse_known_args()
    
    tm = taskManager(opts)
    try:
        tm.run()
    finally:
        tm.die()
