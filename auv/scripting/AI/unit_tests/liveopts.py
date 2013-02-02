#!/usr/bin/env python2.7
import unittest
import time
import utils.test_process as tp
import os.path
import test_script
import AI.options
import cauv.messaging as msg

test_dir = os.path.split(os.path.abspath(__file__))[0]

class TestAIExecutables(tp.ProcessTestCase):
    def test_script(self):
        with tp.TempProcessEnv(True) as env:
            env.start_process([os.path.join(test_dir, 'test_script.py'),
                               '--wait_seconds', '2', '--task-name', 'test'])

            time.sleep(1)
            default_opts = test_script.Script.DefaultOptions()
            default_opts.succeed = True
            default_opts.nest.c = 6434576
            opts = default_opts.to_flat_dict()
            env.node.send(msg.SetTaskStateMessage("test", [], {}, opts))
            time.sleep(1.5)

            log = env.get_log('log/test_script.py.log')
            self.relevant_log = log
            self.assertTrue(tp.grep_log(log, "Script completed successfully"))
            self.assertTrue(tp.grep_log(log, "Nested C: 6434576"))

if __name__ == "__main__":
    unittest.main()
