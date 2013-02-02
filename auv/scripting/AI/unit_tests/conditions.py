#!/usr/bin/env python2.7
import unittest
import AI.conditions

class TestAIState(object):
    def latitude(self):
        return 0
    def longitude(self):
        return 0
    def depth(self):
        return 0
    def detector_fired(self, detector):
        return True
    def task_state(self, task):
        return True

class TestMissionLoad(unittest.TestCase):
    def test_get_conditions_state(self):
        ai_state = TestAIState()
        for condition in AI.conditions.get_conditions().values():
            c = condition(ai_state)
            bool(c.get_state())

if __name__ == "__main__":
    unittest.main()
