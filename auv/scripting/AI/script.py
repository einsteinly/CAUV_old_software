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
        proc.Proc.__init__(self)
        self.options = self.get_options()
        self.task_name = self.options._task_name
        self.node = node.Node(self.__class__.__name__ + "Script")
        self.auv = control.AUV(self.node)
        self.node.subMessage(msg.SetTaskStateMessage())
        self.node.addObserver(self)

    def onSetTaskStateMessage(self, m):
        if m.taskId != self.task_name:
            return
        debug("Setting options")
        opts = BoostMapToDict(m.scriptOptions)
        self.options.from_flat_dict(opts)

    @classmethod
    def entry(cls):
        cls.get_options()
        instance = cls()

        ret = False
        try:
            ret = instance.run()
        finally:
            instance.auv.stop()
            instance.unload_pipeline("")
            info("Script pipelines cleaned up")
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
