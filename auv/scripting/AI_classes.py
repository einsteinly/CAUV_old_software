import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import threading
import cPickle
import time

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
    
def is_external(f):
    try:
        return f.ext_func
    except AttributeError:
        return False

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
            if is_external(getattr(self,message[2])):
                try:
                    getattr(self,message[2])(*message[3], **message[4])
                except Exception as exc:
                    error("Error occured because of message: %s" %(str(message)))
                    raise exc
            else:
                error("AI message %s did not call a valid function (make sure the function is declared as an external function" %(str(message)))

class fakeAUVfunction():
    def __init__(self, script, attr):
        self.script = script
        self.attr = attr
    def __call__(self, *args, **kwargs):
        self.script.ai.auv_control.auv_command(self.script.process_name, self.attr, *args, **kwargs)

class fakeAUV(messaging.BufferedMessageObserver):
    #TODO
    #needs to respond to control overriding commands
    def __init__(self, script):
        self.script = script #passing the script saves on the number of AI_process, as fakeAUV can now call other processes through the script
        messaging.BufferedMessageObserver.__init__(self)
        self.script.node.join("telemetry")
        self.script.node.addObserver(self)
        self.current_bearing = None
        self.current_depth = None
        self.current_pitch = None
        self.bearingCV = threading.Condition()
        self.depthCV = threading.Condition()
        self.pitchCV = threading.Condition()
        
    def onTelemetryMessage(self, m):
        #self.bearing = m.orientation.yaw
        #print "message"
        self.current_bearing = m.orientation.yaw
        self.current_depth = m.depth
        self.current_pitch = m.orientation.pitch
        self.bearingCV.acquire()
        self.depthCV.acquire()
        self.pitchCV.acquire()
        self.bearingCV.notifyAll()
        self.depthCV.notifyAll()
        self.pitchCV.notifyAll()
        self.bearingCV.release()
        self.depthCV.release()
        self.pitchCV.release()
        
    def bearingAndWait(self, bearing, epsilon = 5, timeout = 30):
        startTime = time.time()
        self.bearing(bearing)
        while time.time() - startTime < timeout:
            if self.current_bearing == None or min((bearing - self.current_bearing) % 360, (self.current_bearing - bearing) % 360) > epsilon:
                #print 'bearing waiting'
                self.bearingCV.acquire()
                self.bearingCV.wait(timeout - time.time() + startTime)
                self.bearingCV.release()
            else:
                return True
        return False
        
    def depthAndWait(self, depth, epsilon = 5, timeout = 30):
        startTime = time.time()
        self.depth(depth)
        while time.time() - startTime < timeout:
            if self.current_depth == None or abs(depth - self.current_depth) > epsilon:
                self.depthCV.acquire()
                self.depthCV.wait(timeout - time.time() + startTime)
                self.depthCV.release()
            else:
                return True
        return False

    def pitchAndWait(self, pitch, epsilon = 5, timeout = 30):
        startTime = time.time()
        self.pitch(pitch)
        while time.time() - startTime < timeout:
            if self.current_pitch == None or min((pitch - self.current_pitch) % 360, (self.current_pitch - pitch) % 360) > epsilon:
                self.pitchCV.acquire()
                self.pitchCV.wait(timeout - time.time() + startTime)
                self.pitchCV.release()
            else:
                return True
        return False
        
    def __getattr__(self, attr):
        return fakeAUVfunction(self.script, attr)
        
class aiScript(aiProcess):
    def __init__(self, script_name):
        aiProcess.__init__(self, script_name)
        self.exit_confirmed = threading.Event()
        self.script_name = script_name
        self.auv = fakeAUV(self)
    def notify_exit(self, exit_status):
        for x in range(5):
            self.ai.task_manager.on_script_exit(exit_status)
            if self.exit_confirmed.wait(1.0):
                return
        error("Task manager failed to acknowledge script "+self.script_name+" exit")
        return
    @external_function
    def confirm_exit(self):
        self.exit_confirmed.set()
        
class aiDetector(messaging.BufferedMessageObserver):
    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        self.node = node
        self.node.addObserver(self)
        self.detected = False
    def process(self):
        """
        This should define a method to do any intensive (ie not on message) processing
        """
        pass
    def die(self):
        self.node.removeObserver(self)

class aiCondition():
    """
    Basic condition that can be used for tasks, ie they may be set via task_manager from any other process
    and then when they change task manager checks to see if the conditions for any task have been met
    """
    def __init__(self, name):
        self.name = name
        self.state_lock = threading.Lock()
        self.state = False
    def register(self, task_manager):
        with task_manager.task_lock:
            while self.name in task_manager.conditions:
                error('A condition named '+self.name+' already exists. Trying to register with new name')
                self.name = self.name+'1'
            task_manager.conditions[self.name]=self
    def set_state(self, state):
        with self.state_lock:
            self.state = state
    def get_state(self):
        with self.state_lock:
            state = self.state
        return state
    def deregister(self, task_manager):
        with task_manager.task_lock:
            task_manager.conditions.pop(self.name)
        
class timeCondition(aiCondition):
    """
    This condition only remains true for a certain time
    """
    def __init__(self, name, default_time=0):
        self.name = name
        self.default_time = default_time
        self.state_lock = threading.Lock()
        self.timeout = None
    def set_state(self, state, time=None):
        with self.state_lock:
            if state:
                self.timeout=time.time()+(time if time else self.default_time)
            else:
                self.timeout = None
    def get_state(self):
        with self.state_lock:
            state = self.timeout>time.time()
        return state
    
class detectorCondition(aiCondition):
    """
    This condition relies on the state of a detector
    """
    def __init__(self, name, detector_name):
        aiCondition.__init__(self, name)
        self.detector_name = detector_name
    def register(self, task_manager):
        aiCondition.register(self, task_manager)
        #We need to tell the task manager to setup the detector, and redirect messages to this condition
        task_manager.add_detector(self.detector_name, self)
    def deregister(self, task_manager):
        task_manager.remove_detector(self.detector_name, self)
        aiCondition.deregister(self, task_manager)

class aiTask():
    def __init__(self, script_name, priority, running_priority=None, conditions=[]):
        self.script_name = script_name
        self.conditions = conditions
        self.priority = priority
        self.running_priority = running_priority if running_priority else priority
    def register(self, task_manager):
        task_manager.active_tasks.append(self)
        for condition in self.conditions:
            condition.register(task_manager)
    def is_available(self):
        for condition in self.conditions:
            if not condition.get_state():
                return False
        return True
