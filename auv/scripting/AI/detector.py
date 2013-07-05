from cauv.debug import debug, warning, error, info
from cauv import messaging as msg
import proc
import cauv.node as node

class Detector(proc.Proc):
    def __init__(self):
        proc.Proc.__init__(self, self.__class__.__name__ + "Detector")
        self.options = self.get_options()
        self.task_name = self.options._task_name
        self.debug = self.Debug()
        self.node.subMessage(msg.SetConditionStateMessage())
        
    def report(self):
        self.node.send(msg.ConditionDebugStateMessage(self.task_name, self.debug.to_boost_dict()))

    def fire(self, timeout):
        if timeout == 0:
            info("Detector stopped firing")
        else:
            info("Detector fired with timout of {} seconds".format(int(timeout)))
        warning("Auto conversion to int timeout in fire, please fix.")
        #TODO fix
        self.node.send(msg.DetectorFiredMessage(self.task_name, int(timeout)))
   
    def onSetConditionStateMessage(self, m):
        if m.taskId != self.task_name:
            return
        debug("Setting options")
        self.options.from_boost_dict(m.conditionOptions)

    @classmethod
    def entry(cls):
        cls.get_options()
        instance = cls()

        ret = False
        try:
            ret = instance.run()
        finally:
            instance.cleanup()
