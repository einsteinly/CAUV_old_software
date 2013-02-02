#!/usr/bin/env python2.7
import unittest
import test_script
import script_library
script_library.test = test_script

import AI.mission
import AI.conditions
import AI.tasks

expected_condition_save = """!condition/State
name: State_1
options: {state: false}
"""

expected_script_save = """!script/test
debug_message: Testing
nest: {a: 1, b: 2, c: 3}
succeed: false
wait_seconds: 1
"""

test_load = """!task
conditions:
- !condition/State
  name: State_Test
  options: {state: true}
name: test_4
options: {crash_limit: 5, frequency_limit: 1, priority: 1, running_priority: 1}
script: !script/test {debug_message: Changed, succeed: true, wait_seconds: 1}
"""

expected_full_save = """!task
conditions:
- !condition/State
  name: State_1
  options: {state: false}
name: test_1
options: {crash_limit: 5, frequency_limit: 30, priority: 1, running_priority: 2}
script: !script/test
  debug_message: Testing
  nest: {a: 1, b: 2, c: 3}
  succeed: false
  wait_seconds: 1
"""

expected_state_save = """test_4: {active: false, crash_count: 0, failed: false, last_called: 0, paused: false,
  succeeded: false}
"""

test_state_load = """test_4: {active: true, crash_count: 3, last_called: 75438354, failed: false, paused: false, succeeded: false}
"""

class TestMissionLoad(unittest.TestCase):
    def test_full_save(self):
        t = AI.tasks.Task([AI.conditions.StateCondition(None)], AI.tasks.TaskScript("test"))
        save = AI.mission.dump(t)
        self.assertMultiLineEqual(save, expected_full_save)

    def test_all_condition_load_save(self):
        for condition in AI.conditions.get_conditions().values():
            c = condition(None)
            dump = AI.mission.dump(c)
            cc = AI.mission.load(dump)

    def test_condition_save(self):
        ss = AI.conditions.StateCondition(None)
        save = AI.mission.dump(ss)
        self.assertMultiLineEqual(save, expected_condition_save)

    def test_script_save(self):
        ss = AI.tasks.TaskScript("test")
        save = AI.mission.dump(ss)
        self.assertMultiLineEqual(save, expected_script_save)

    def test_load(self):
        t = AI.mission.load(test_load)
        self.assertEqual(t.name, "test_4")
        self.assertEqual(t.script.name, "test")
        self.assertEqual(t.script.options.debug_message, "Changed")
        self.assertEqual(t.script.options.succeed, True)
        self.assertEqual(t.script.options.wait_seconds, 1)
        self.assertEqual(t.options.frequency_limit, 1)
        self.assertEqual(len(t.conditions), 1)
        self.assertEqual(t.conditions[0].__class__, AI.conditions.StateCondition)
        self.assertEqual(t.conditions[0].options.state, True)

    def test_state_save(self):
        m = {}
        t = AI.mission.load(test_load)
        m[t.name] = t
        state = AI.mission.dump_state(m)
        self.assertMultiLineEqual(state, expected_state_save)

    def test_state_load(self):
        m = {}
        t = AI.mission.load(test_load)
        m[t.name] = t
        AI.mission.load_state(test_state_load, m)
        self.assertEqual(t.state.active, True)
        self.assertEqual(t.state.crash_count, 3)
        self.assertEqual(t.state.last_called, 75438354)
        self.assertEqual(t.state.paused, False)
        self.assertEqual(t.state.succeeded, False)

if __name__ == "__main__":
    unittest.main()
