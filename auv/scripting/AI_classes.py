import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import threading
import cPickle

#ai messages are of form message is (to, from, function_name, args, kwargs)

class aiForeignFunction():
    def __init__(self, node, calling_process, process, function):
        self.node = node
        self.calling_process = calling_process
        self.process = process
        self.function = function
    def __call__(self, *args, **kwargs):
        message = cPickle.dumps((self.process, self.calling_process, self.function, args, kwargs))
        self.node.send(messaging.AIMessage(message), "ai")

class aiForeignProcess():
    def __init__(self, node, calling_process, process):
        self.node = node
        self.calling_process = calling_process
        self.process = process
    def __getattr__(self, function):
        return aiForeignFunction(self.node, self.calling_process, self.process, function)
        
class aiAccess():
    def __init__(self, node, process_name):
        self.node = node
        self.process_name = process_name
    def __getattr__(self, process):
        return aiForeignProcess(self.node, self.process_name, process)
        
#this is actually a decorator, used to declare functions accessible to other processes
def external_function(f):
    f.ext_func = True
    return f

class aiProcess(messaging.BufferedMessageObserver):
    def __init__(self, process_name):
        messaging.BufferedMessageObserver.__init__(self)
        self.node = cauv.node.Node("pyai"+process_name[:4])
        self.node.join("ai")
        self.node.addObserver(self)
        self.process_name = process_name
        self.ai = aiAccess(self.node, self.process_name)
    def onAIMessage(self, m):
        message = cPickle.loads(m.msg)
        if message[0] == self.process_name: #this is where the to string appears in the cpickle output
            message = cPickle.loads(m.msg)
            if hasattr(self.__getattribute__(message[2]), 'ext_func'):
                try:
                    self.__getattribute__(message[2])(*message[3], **message[4])
                except Exception as exc:
                    error("Error occured because of message: %s" %(str(message)))
                    raise exc
            else:
                error("AI message %s did not call a valid function (make sure the function is declared as an external function" %(str(message)))
            
class aiScript(aiProcess):
    def __init__(self, script_name):
        aiProcess.__init__(self, "auv_script")
        self.script_name = script_name
        self.exit_confirmed = threading.Event()
    def exit(self, exit_status):
        for x in range(5):
            self.ai.task_manager.on_script_exit(exit_status)
            if self.exit_confirmed.wait(1.0):
                return
        error("Task manager failed to acknowledge script "+self.script_name+" exit")
        return
    @external_function
    def confirm_exit(self):
        self.exit_confirmed.set()
        
class aiDetector():
    def __init__(self, node):
        self.node = node
        self.detected = False
    def process(self):
        """
        This should define a method to do any intensive (ie not on message) processing
        """
        pass
    def die(self):
        """
        aiDetectors MUST define a clearup method. since pretty much all detectors need to disengage from the messaging system, the default method will raise an exception, as a) that way its difficult to foret to define and b) this should stop any detectors as it will force a process restart
        """
        raise Exception("Die method MUST be defined")
