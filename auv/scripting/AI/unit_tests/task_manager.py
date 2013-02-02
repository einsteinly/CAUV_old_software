#!/usr/bin/env python2.7

import time
import socket
import os.path
import unittest

import cauv.node
import cauv.messaging as msg
import utils.test_process as tp
from utils.conv import BoostMapToDict

import test_script
import test_detector
import script_library
import detector_library
script_library.test = test_script
detector_library.test = test_detector

import AI.exe.task_manager

class NodeMock(object):
    def __init__(self, name, args = None, run_now = None):
        self.messages = []

    def send(self, message):
        self.messages.append(message)

    def addObserver(self, observer):
        pass

    def subMessage(self, message):
        pass

    def last_msg(self, m_type):
        for m in reversed(self.messages):
            if isinstance(m, m_type):
                return m

class FakeNodeImport(object):
    def __init__(self):
        self.Node = NodeMock

class FakeOptions(object):
    def __init__(self, restore = False, mission_name = '', mission_save = ''):
        self.restore = restore
        self.mission_name = mission_name
        self.mission_save = mission_save
        self.unpaused = False

class TMTestCase(unittest.TestCase):
    def info(self, message):
        self.info_messages.append(message)
    def debug(self, message, level = 0):
        self.debug_messages.append(message)
    def error(self, message):
        self.assertTrue(False, message)
    def warning(self, message):
        self.assertTrue(False, message)

    def setUp(self):
        self.realimport = AI.exe.task_manager.cauv.node
        AI.exe.task_manager.cauv.node = FakeNodeImport()
        self.realinfo = AI.exe.task_manager.info
        self.realdebug = AI.exe.task_manager.debug
        self.realerror = AI.exe.task_manager.error
        self.realwarning = AI.exe.task_manager.warning
        self.info_messages = []
        self.debug_messages = []
        AI.exe.task_manager.info = self.info
        AI.exe.task_manager.debug = self.debug
        AI.exe.task_manager.warning = self.warning
        AI.exe.task_manager.error = self.error

    def tearDown(self):
        AI.exe.task_manager.cauv.node = self.realimport
        AI.exe.task_manager.info = self.realinfo
        AI.exe.task_manager.debug = self.realdebug
        AI.exe.task_manager.warning = self.realwarning
        AI.exe.task_manager.error = self.realerror

    def get_tm(self):
        return AI.exe.task_manager.TaskManager(FakeOptions())

    def add_test_condition(self, tm):
        tm.onAddConditionMessage.func(msg.AddConditionMessage("State"))
        cond_name = tm.node.messages[-1].conditionId
        self.assertTrue(cond_name.startswith("State"))
        tm.onAddTaskMessage.func(msg.AddTaskMessage("test"))
        task_name = tm.node.messages[-1].taskId
        self.assertTrue(task_name.startswith("test"))
        tm.onSetTaskStateMessage.func(
            msg.SetTaskStateMessage(
                task_name,
                [cond_name],
                {},{}))
        condition = tm.conditions[cond_name]
        self.assertIn(condition, tm.tasks[task_name].conditions)
        return cond_name, task_name

class TestGUIMessages(TMTestCase):
    def test_tm_init(self):
        tm = self.get_tm()
        initial_messages = {m.__class__.__name__ for m in tm.node.messages}
        task_types = tm.node.last_msg(msg.TaskTypesMessage)
        self.assertIsNotNone(task_types)
        self.assertIn("test", task_types.typeNames)
        self.assertIsNotNone(tm.node.last_msg(msg.ConditionTypesMessage))

    def test_add_task(self):
        tm = self.get_tm()
        tm.onAddTaskMessage.func(msg.AddTaskMessage("test"))
        self.assertIn("test", {k.split("_")[0] for k in tm.tasks})

    def test_remove_task(self):
        tm = self.get_tm()
        cond_name, task_name = self.add_test_condition(tm)
        tm.onRemoveTaskMessage.func(msg.RemoveTaskMessage(task_name))

    def test_add_condition(self):
        tm = self.get_tm()
        tm.onAddConditionMessage.func(msg.AddConditionMessage("State"))
        self.assertIn("State", {k.split("_")[0] for k in tm.conditions})
        last_msg = tm.node.messages[-1]
        self.assertTrue(last_msg.conditionId.startswith("State"))

    def test_link_condition(self):
        tm = self.get_tm()
        self.add_test_condition(tm)

    def test_remove_condition(self):
        tm = self.get_tm()
        cond_name, task_name = self.add_test_condition(tm)
        condition = tm.conditions[cond_name]
        tm.onRemoveConditionMessage.func(msg.RemoveConditionMessage(cond_name))
        self.assertNotIn(cond_name, tm.conditions)
        self.assertNotIn(condition, tm.tasks[task_name].conditions)

    def test_set_task_options(self):
        tm = self.get_tm()
        tm.onAddTaskMessage.func(msg.AddTaskMessage("test"))
        task_state = tm.node.last_msg(msg.TaskStateMessage)
        task_name = task_state.taskId
        options = task_state.taskOptions
        self.assertIn('crash_limit', options)
        self.assertIn('priority', options)
        options['crash_limit'] = 666
        tm.onSetTaskStateMessage.func(
            msg.SetTaskStateMessage(
                task_name,
                [],
                options, {}))
        self.assertEqual(tm.tasks[task_name].options.crash_limit, 666)
        task_state = tm.node.last_msg(msg.TaskStateMessage)
        options = BoostMapToDict(task_state.taskOptions)
        self.assertEqual(options['crash_limit'], 666)

    def test_set_condition_options(self):
        tm = self.get_tm()
        cond_name, task_name = self.add_test_condition(tm)
        condition_state = tm.node.last_msg(msg.ConditionStateMessage)
        options = condition_state.conditionOptions
        self.assertIn('state', options)
        options['state'] = True
        tm.onSetConditionStateMessage.func(
            msg.SetConditionStateMessage(
                cond_name,
                options))
        self.assertTrue(tm.conditions[cond_name].options.state)
        condition_state = tm.node.last_msg(msg.ConditionStateMessage)
        options = condition_state.conditionOptions
        self.assertTrue(options['state'])

    def test_control(self):
        tm = self.get_tm()
        self.add_test_condition(tm)
        self.assertTrue(tm.all_paused)
        tm.onAIControlMessage.func(msg.AIControlMessage(msg.AICommand.ResumeAll))
        self.assertFalse(tm.all_paused)
        tm.onAIControlMessage.func(msg.AIControlMessage(msg.AICommand.PauseAll))
        self.assertTrue(tm.all_paused)
        with tp.TempProcessEnv() as env:
            save_dir = os.path.join(env.dirname, "save")
            os.mkdir(save_dir)
            tm.mission_name = save_dir + "/default"
            tm.onAIControlMessage.func(msg.AIControlMessage(msg.AICommand.Save))
            self.assertTrue(os.listdir(save_dir))

test_simple_load = """- !task
  conditions:
  - !condition/State
    name: State_44
    options: { state: true}
  name: test_44
  options: {crash_limit: 1, frequency_limit: 0.2, priority: 1, running_priority: 2}
  script: !script/test
    succeed: true
"""

test_priority_change = """
- !task
  conditions:
  - !condition/State
    name: State_low_prio
    options: { state: false }
  name: low_prio
  options: { priority: 1, frequency_limit: 0, running_priority: 1 }
  script: !script/test
    succeed: true
- !task
  conditions:
  - !condition/State
    name: State_high_prio
    options: { state: false }
  name: high_prio
  options: { priority: 2, frequency_limit: 0, running_priority: 2 }
  script: !script/test
    succeed: true
"""

test_crashing = """
- !task
  conditions:
  - !condition/State
    name: state_crashy
    options: { state: true }
  name: crashy
  options: { crash_limit: 3, frequency_limit: 0 }
  script: !script/test
    succeed: true
"""

test_failure = """ 
- !task
  conditions:
  - !condition/State
    name: state_fail
    options: { state: true }
  name: fail
  options: { crash_limit: 3, frequency_limit: 0 }
  script: !script/test
    succeed: true
"""

test_detector = """
- !task
  conditions:
  - !condition/test
    name: detector
    options: {}
  name: detector_t
  options: {}
  script: !script/test
    succeed: true
"""

class TestTaskManager(TMTestCase):
    def test_start_script(self):
        tm = self.get_tm()
        tm.onAddTaskMessage.func(msg.AddTaskMessage("test"))
        task_state = tm.node.last_msg(msg.TaskStateMessage)
        task_name = task_state.taskId
        tm.onAIControlMessage.func(msg.AIControlMessage(msg.AICommand.ResumeAll))
        tm.start_script(tm.tasks[task_name])
        edit_process = tm.node.last_msg(msg.EditProcessMessage)
        start_process = tm.node.last_msg(msg.ProcessControlMessage)
        self.assertEqual(edit_process.command[0], "python2.7")
        self.assertTrue(edit_process.command[1].endswith("test_script.pyc"))
        self.assertIsNot(start_process, None)

    def test_conditions_start(self):
        tm = self.get_tm()
        tm.all_paused = False
        cond_name, task_name = self.add_test_condition(tm)
        tm.process_periodic.func()
        #Shouldn't do anything yet
        self.assertIsNone(tm.node.last_msg(msg.EditProcessMessage))
        tm.conditions[cond_name].options.state = True
        tm.process_periodic.func()
        self.assertIsNotNone(tm.node.last_msg(msg.EditProcessMessage))
        self.assertIsNotNone(tm.node.last_msg(msg.ProcessControlMessage))

    def test_load_mission(self):
        tm = self.get_tm()
        tm.load_mission(test_simple_load)
        self.assertIn("test_44", tm.tasks)
        self.assertIn("State_44", tm.conditions)

    def test_priority(self):
        tm = self.get_tm()
        tm.load_mission(test_priority_change)
        tm.all_paused = False
        tm.conditions['State_low_prio'].options.state = True
        tm.process_periodic.func()
        self.assertIn("low_prio", [t.name for t in tm.active_tasks()])
        self.assertNotIn("high_prio", [t.name for t in tm.active_tasks()])
        tm.conditions['State_high_prio'].options.state = True
        tm.process_periodic.func()
        self.assertNotIn("low_prio", [t.name for t in tm.active_tasks()])
        self.assertIn("high_prio", [t.name for t in tm.active_tasks()])
        tm.conditions['State_high_prio'].options.state = False
        tm.process_periodic.func()
        self.assertIn("low_prio", [t.name for t in tm.active_tasks()])
        self.assertNotIn("high_prio", [t.name for t in tm.active_tasks()])

    def test_crashing(self):
        tm = self.get_tm()
        tm.load_mission(test_crashing)
        tm.all_paused = False
        for i in range(4):
            tm.process_periodic.func()
            self.assertIn("crashy", [t.name for t in tm.active_tasks()])
            if i in (1,2):
                tm.onProcessEndedMessage.func(msg.ProcessEndedMessage(socket.gethostname(), "ai_script/crashy"))
            else:
                tm.onScriptExitMessage.func(msg.ScriptExitMessage("crashy", msg.ScriptExitStatus.TempFailure))
        tm.process_periodic.func()
        self.assertNotIn("crashy", [t.name for t in tm.active_tasks()])

    def test_script_fail(self):
        tm = self.get_tm()
        tm.load_mission(test_failure)
        tm.all_paused = False
        tm.process_periodic.func()
        self.assertIn("fail", [t.name for t in tm.active_tasks()])
        tm.onScriptExitMessage.func(msg.ScriptExitMessage("fail", msg.ScriptExitStatus.PermFailure))
        tm.process_periodic.func()
        self.assertNotIn("fail", [t.name for t in tm.active_tasks()])

    def test_detector_condition(self):
        tm = self.get_tm()
        tm.load_mission(test_detector)
        tm.all_paused = False
        self.assertIsNotNone(tm.node.last_msg(msg.EditProcessMessage))
        self.assertIsNotNone(tm.node.last_msg(msg.ProcessControlMessage))
        tm.process_periodic.func()
        self.assertNotIn("detector_t", [t.name for t in tm.active_tasks()])
        tm.onDetectorFiredMessage.func(msg.DetectorFiredMessage("detector", 10000))
        tm.process_periodic.func()
        self.assertIn("detector_t", [t.name for t in tm.active_tasks()])

if __name__ == "__main__":
    unittest.main()
