#!/usr/bin/env python2.7
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

import os
import yaml
import time
import shelve
import socket
import inspect
import argparse
import datetime
import traceback
import itertools
import subprocess
import collections

from collections import deque

import AI.conditions
import AI.tasks
import AI.options
import AI.mission
import script_library
import detector_library

import utils.event as event

"""          
task manager auto generates a list of what it should be running from these 'tasks',
basically looking for these tasks and then running appropriate scripts
"""          

TASK_CHECK_INTERVAL = 1
STATE_SAVE_INTERVAL = 30

class AIState(object):
    def __init__(self, task_manager):
        self.manager = task_manager
    def latitude(self):
        return 0
    def longitude(self):
        return 0
    def depth(self):
        return 0
    def detector_fired(self, detector):
        return self.manager.detector_fire_timeouts[detector] > time.time()
    def task_state(self, task):
        return True

class NameDict(dict):
    """Dictionary which allows lookup by an objects name, and nothing else"""
    #This is to enforce the lookup for tasks and conditions
    def __setitem__(self, key, value):
        if value.name != key:
            raise ValueError("Name does not match key!")
        super(NameDict, self).__setitem__(key, value)

    def add(self, value):
        self[value.name] = value

class TaskManager(event.EventLoop, messaging.MessageObserver):
    #SETUP FUNCTIONS
    def __init__(self, opts):
        super(TaskManager, self).__init__()
        self.node = cauv.node.Node("task_manager")
        self.node.subMessage(messaging.AddTaskMessage())
        self.node.subMessage(messaging.RemoveTaskMessage())
        self.node.subMessage(messaging.SetTaskStateMessage())
        self.node.subMessage(messaging.AddConditionMessage())
        self.node.subMessage(messaging.RemoveConditionMessage())
        self.node.subMessage(messaging.SetConditionStateMessage())
        self.node.subMessage(messaging.RequestAIStateMessage())
        self.node.subMessage(messaging.ScriptControlMessage())
        self.node.subMessage(messaging.AIControlMessage())
        self.node.subMessage(messaging.ProcessEndedMessage())
        self.node.subMessage(messaging.DetectorFiredMessage())
        self.node.subMessage(messaging.ScriptExitMessage())
        self.all_paused = not opts.unpaused
        self.ai_state = AIState(self)
        #Tasks - list of tasks that (in right conditions) should be called
        self.tasks = NameDict()
        self.conditions = NameDict()
        self.detector_fire_timeouts = collections.defaultdict(int)
        #currently running task
        self.last_available_tasks = []
        self.last_active_tasks = []
        self.mission_name = opts.mission_name
        if opts.mission_save:
            self.load_mission(open(opts.mission_save))
        self.gui_send_all()
        self.node.addObserver(self)

    def load_mission(self, stream):
        task_list = AI.mission.load(stream, ai_state = self.ai_state)
        for t in task_list:
            self.tasks.add(t)
            for c in t.conditions:
                self.conditions.add(c)
                if isinstance(c, AI.conditions.DetectorCondition):
                    self.start_detector(c)

    @event.event_func
    def onAddTaskMessage(self, msg):
        try:
            task_script = AI.tasks.TaskScript(msg.taskType)
        except KeyError:
            error("Task type {} does not exist!".format(msg.taskType))
            return
        task = AI.tasks.Task([], task_script)
        self.tasks.add(task)
        debug("Tasks: {}".format(self.tasks))
        self.gui_update_task(task)

    @event.event_func
    def onRemoveTaskMessage(self, msg):
        try:
            task = self.tasks[msg.taskId]
        except KeyError:
            error("Asked to remove non-existent task {}".format(msg.taskId))
            return
        self.stop_script(task)
        del self.tasks[task.name]
        self.node.send(messaging.TaskRemovedMessage(msg.taskId))

    @event.event_func
    def onSetTaskStateMessage(self, msg):
        try:
            task = self.tasks[msg.taskId]
        except KeyError:
            error("Task {} does not exist!".format(msg.taskId))
            return
        task_conditions = []
        for c in msg.conditionIds:
            try:
                task_conditions.append(self.conditions[c])
            except KeyError:
                error("Condition {} does not exist!".format(c))
        task.conditions = task_conditions
        task.options.from_boost_dict(msg.taskOptions)
        task.script.options.from_boost_dict(msg.scriptOptions)
        self.gui_update_task(task)

    @event.event_func
    def onAddConditionMessage(self, msg):
        try:
            Condition = AI.conditions.get_conditions()[msg.conditionType]
        except KeyError:
            error("Condition type {} does not exist!".format(msg.conditionType))
            return
        condition = Condition(self.ai_state)
        self.conditions.add(condition)
        self.gui_update_condition(condition)
        debug("Conditions: {}".format(self.conditions))
        if isinstance(condition, AI.conditions.DetectorCondition):
            self.start_detector(condition)

    @event.event_func
    def onRemoveConditionMessage(self, msg):
        try:
            condition = self.conditions[msg.conditionId]
        except KeyError:
            error("Condition {} does not exist!".format(msg.conditionId))
            return
        for task in self.tasks.values():
            try:
                task.conditions.remove(condition)
            except ValueError:
                pass
        if isinstance(condition, AI.conditions.DetectorCondition):
            self.stop_detector(condition)
        del self.conditions[msg.conditionId]
        self.node.send(messaging.ConditionRemovedMessage(msg.conditionId))

    @event.event_func
    def onSetConditionStateMessage(self, msg):
        try:
            condition = self.conditions[msg.conditionId]
        except KeyError:
            error("Condition {} does not exist!".format(msg.conditionId))
            return

        condition.options.from_boost_dict(msg.conditionOptions)
        self.gui_update_condition(condition)

    @event.event_func
    def onScriptControlMessage(self, msg):
        debug("ScriptControlMessage not currently implemented")
        pass

    @event.event_func
    def onAIControlMessage(self, msg):
        command = msg.command.name
        if command == 'PauseAll':
            self.all_paused = True
            for task in self.tasks.values():
                if task.state.active:
                    self.stop_script(task)
            info("AI Paused")
        elif command == 'ResumeAll':
            self.all_paused = False
            info("AI Unpaused")
        elif command == 'Save':
            save_file = "{}_save_{}.yaml".format(self.mission_name, datetime.datetime.now().isoformat())
            info("Saving mission to {}".format(save_file))
            try:
                AI.mission.dump(self.tasks.values(), open(save_file, 'w'))
            except IOError:
                error('Could not save mission')
                error(traceback.format_exc().encode('ascii', 'ignore'))                        

    @event.event_func
    def onRequestAIStateMessage(self, msg):
        self.gui_send_all()

    @event.event_func
    def onDetectorFiredMessage(self, msg):
        self.detector_fire_timeouts[msg.conditionId] = time.time() + float(msg.timeout)
        self.gui_update_condition(self.conditions[msg.conditionId])
        
    @event.event_func
    def onProcessEndedMessage(self, msg):
        if not msg.process.startswith('ai_script'):
            return
        try:
            task_name = msg.process.split('/')[1]
        except IndexError:
            warning("Badly formatted AI process name")
            return
        try:
            task = self.tasks[task_name]
            info("Script for task {} exited".format(task.name))
            self.cleanup_task_pl(task)
            if task.state.active:
                task.crashed()
        except KeyError:
            #can happen when removing tasks
            warning("Process for task {} ended, but task not present".format(task_name))

    @event.event_func
    def onScriptExitMessage(self, msg):
        try:
            task = self.tasks[msg.task]
        except KeyError:
            error("Received exit for task {} which doesn't exist!".format(msg.task))
        self.stop_script(task)
        if msg.status == messaging.ScriptExitStatus.Success:
            task.succeeded()
            info("Task {} succeeded".format(task.name))
        if msg.status == messaging.ScriptExitStatus.TempFailure:
            task.crashed()
        if msg.status == messaging.ScriptExitStatus.PermFailure:
            task.failed()

    #MESSAGES TO GUI
    def gui_update_task(self, task):
        self.node.send(messaging.TaskStateMessage(task.name,
                [t.name for t in task.conditions],
                task.options.to_boost_dict(),
                task.script.options.to_boost_dict(),
                task.state.active))

    def gui_update_condition(self, condition):
        self.node.send(messaging.ConditionStateMessage(
            condition.name,
            condition.options.to_boost_dict(),
            condition.get_state()))
        debug("Condition {} has state {}".format(condition.name, condition.get_state()))

    def gui_send_all(self):
        #send type info
        task_types = script_library.index
        condition_types = AI.conditions.get_conditions().keys()
        self.node.send(messaging.TaskTypesMessage(task_types))
        self.node.send(messaging.ConditionTypesMessage(condition_types))

        #send conditions (first, since tasks depend on conditions)
        for condition in self.conditions.itervalues():
            self.gui_update_condition(condition)
        #send tasks
        for task in self.tasks.itervalues():
            self.gui_update_task(task)

    def cleanup_task_pl(self, task):
        self.node.send(cauv.messaging.ClearPipelineMessage("ai/{}".format(task.name)))

    def start_proc(self, name, cmd):
        self.node.send(
            messaging.EditProcessMessage(
                socket.gethostname(),
                name, cmd,
                True,
                "", [], -101)) #don't care about node id, no prereqs, magic report death restart value
        self.node.send(
            messaging.ProcessControlMessage(
                messaging.ProcessCommand.Start,
                socket.gethostname(), name))

    def stop_proc(self, name):
        self.node.send(
            messaging.ProcessControlMessage(
                messaging.ProcessCommand.Stop,
                socket.gethostname(),
                name))

    def start_detector(self, detector):
        detector_path = inspect.getfile(detector.Detector)
        detector_opts = detector.options.to_cmd_opts()
        detector_cmd = ["python2.7", detector_path, '-t', detector.name] + detector_opts
        self.start_proc("ai_detect/{}".format(detector.name), detector_cmd)

    def stop_detector(self, detector):
        self.stop_proc("ai_detect/{}".format(detector.name))

    def start_script(self, task):
        if self.all_paused:
            return
        info("Starting script {} for task {}".format(task.script.name, task.name))
        script_path = inspect.getfile(task.script.script_class)
        script_opts = task.script.options.to_cmd_opts()
        script_cmd = ["python2.7", script_path, '-t', task.name, '-b'] + script_opts
        self.start_proc("ai_script/{}".format(task.name), script_cmd)
        task.started()
        self.gui_update_task(task)

    def stop_script(self, task):
        self.stop_proc("ai_script/{}".format(task.name))
        info('Stopping script {} for task {}'.format(task.script.name, task.name))
        task.stopped()
        self.cleanup_task_pl(task)
        self.gui_update_task(task)

    @event.repeat_event(STATE_SAVE_INTERVAL, True)
    def save_state(self):
        pass
        
    def active_tasks(self):
        return [t for t in self.tasks.values() if t.state.active]

    #--function run by periodic loop--
    @event.repeat_event(TASK_CHECK_INTERVAL, True)
    def process_periodic(self):
        if self.all_paused:
            return
        #check tasks, run as appropriate
        available_tasks = [t for t in self.tasks.values() if t.is_available()]
        available_tasks.sort(key = lambda x: (-x.priority(), x.name))
        active_tasks = self.active_tasks()
        if self.last_available_tasks != available_tasks or self.last_active_tasks != active_tasks:
            self.last_available_tasks = available_tasks
            task_list = ', '.join(["{} ({})".format(t.name, t.priority()) for t in available_tasks])
            info("Available tasks: {}".format(task_list))
            info("Triggered conditions: {}".format([c.name for c in self.conditions.values() if c.get_state()]))
            to_run = available_tasks[0] if available_tasks else None
            for task in self.tasks.values():
                if task.state.active and task is not to_run:
                    self.stop_script(task)
                if task is to_run and not task.state.active:
                    self.start_script(task)
            self.last_active_tasks = self.active_tasks()
            self.gui_send_all()

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-s', '--state', default = '', help="file to load/save state from")
    p.add_argument('-m', '--mission_name', default = 'mission', help="try and resume from last saved state")
    p.add_argument('-f', '--mission_save', default = '', help="load saved state")
    p.add_argument('-g', '--unpaused', action="store_true", help="Start unpaused")
    opts, args = p.parse_known_args()
    
    tm = TaskManager(opts)
    tm.run()
