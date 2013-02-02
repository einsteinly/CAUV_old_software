#!/usr/bin/env python2.7

import unittest
import time
import utils.test_process as tp
import os.path

test_dir = os.path.split(os.path.abspath(__file__))[0]

class TestAIExecutables(tp.ProcessTestCase):
    def test_script(self):
        with tp.TempProcessEnv() as env:
            env.start_process([os.path.join(test_dir, 'test_script.py'), '--succeed', 'True', '--wait_seconds', '0'])
            time.sleep(1)
            log = env.get_log('log/test_script.py.log')
            self.assertTrue(tp.grep_log(log, "Script completed successfully"))

    def test_detector(self):
        with tp.TempProcessEnv() as env:
            env.start_process([os.path.join(test_dir, 'test_detector.py'), '--fire', 'True', '--wait_seconds', '0'])
            time.sleep(1)
            log = env.get_log('log/test_detector.py.log')
            self.relevant_log = log
            self.assertTrue(tp.grep_log(log, "Detector fired"))

if __name__ == "__main__":
    unittest.main()
