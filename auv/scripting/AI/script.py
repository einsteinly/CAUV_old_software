from cauv.debug import debug, warning, error, info
import cauv.messaging as msg
import cauv.control as control
import cauv.node as node
import cauv.yamlpipe
import cauv.pipeline
import utils.dirs
from utils.conv import BoostMapToDict
import os.path
import proc

import time
import argparse
import collections

import options

class Script(proc.Proc):
            
    def __init__(self):
        proc.Proc.__init__(self, self.__class__.__name__ + "Script")
        self.options = self.get_options()
        self.task_name = self.options._task_name
        self.auv = control.AUV(self.node)
        self.node.subMessage(msg.SetTaskStateMessage())
        
    def report(self):
        warning("Debug value reporting is currently broken")
        #self.node.send(msg.ScriptStateMessage(self.debug.to_boost_dict()))

    def onSetTaskStateMessage(self, m):
        if m.taskId != self.task_name:
            return
        debug("Setting options")
        self.options.from_boost_dict(m.scriptOptions)

    @classmethod
    def entry(cls):
        cls.get_options()
        instance = cls()
        instance.start_listening()

        ret = False
        try:
            ret = instance.run()
        finally:
            instance.auv.stop()
            instance.cleanup()
        if ret or ret is None:
            status = msg.ScriptExitStatus.Success
            info("Script completed successfully")
        else:
            status = msg.ScriptExitStatus.TempFailure
            info("Script met temporary failure condition")
        if instance.options._broadcast_status:
            while True:
                debug("Broadcasting status message")
                instance.node.send(msg.ScriptExitMessage(status))
                time.sleep(1)
