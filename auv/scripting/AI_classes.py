import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import threading
import cPickle
import time
import traceback

#------AI PROCESSES STUFF------
#ai messages are of form message is (to, from, function_name, args, kwargs)

class CommunicationError(Exception):
    pass

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
#note that it doesn't behave like a normal decorator, the function is extracted from it in the initialisation stages
#be careful, only one external function can run at a time, and the process cannot receive other messages during this
class external_function:
    def __init__(self, f):
        #mark if needs calling_process as well
        if hasattr(f, 'func_code') and (not hasattr(f, 'caller')):
            if 'calling_process' in f.func_code.co_varnames:
                f.caller = True
            else:
                f.caller = False
        self.func = f

def force_calling_process(f):
    f.caller = True
    return f
#The process class is no longer also the message observer
#since it appears that if the metaclass is subclassed, boost
#does not recognise the observer
#(basically, it checks to see that the metaclass of the observer
#class is boost.python.class rather than a subclass of it,
#see http://boost.2283326.n4.nabble.com/Boost-Python-Metaclass-td3308127.html
#it may be fixed in newer versions??)

class aiMessageObserver(messaging.MessageObserver):
    key_index = {'to': 0, 'from': 1, 'function': 2, 'args': 3, 'kwargs': 4}
    def __init__(self, parent_process, function_names):
        messaging.MessageObserver.__init__(self)
        self.parent_process = parent_process
        self.function_names = function_names
    def onAIMessage(self, m):
        debug("onAIMessage in %s: %s" %(self.parent_process.process_name, m.msg), 6)
        message = cPickle.loads(m.msg)
        if message[0] == self.parent_process.process_name: #this is where the to string appears in the cpickle output
            message = cPickle.loads(m.msg)
            if message[2] in self.function_names:
                try:
                    debug("onAIMessage in %s, calling function." %(self.parent_process.process_name, ), 6)
                    if getattr(self.parent_process, message[2]).caller:
                        message[4]['calling_process'] = message[1]
                    getattr(self.parent_process, message[2])(*message[3], **message[4])
                except Exception as exc:
                    error("Error occured because of message: %s" %(str(message)))
                    traceback.print_exc()
            else:
                error("AI message %s did not call a valid function (make sure the function is declared as an external function" %(str(message)))

class aiProcessBase(type):
    def __init__(cls, name, bases, attrs):
        super(aiProcessBase, cls).__init__(name, bases, attrs)
        #list ext funcs in class, and extract function (don't accidentally wipe parent class functions)
        if not hasattr(cls, '_ext_funcs'):
            cls._ext_funcs = []
        if not hasattr(cls, '_objects'):
            cls._objects = []
        for key, attr in attrs.iteritems():
            if isinstance(attr, external_function):
                cls._ext_funcs.append(key)
                setattr(cls, key, attr.func)
            else:
                cls._objects.append(key)
    def __call__(cls, *args, **kwargs):
        inst = cls.__new__(cls, *args, **kwargs)
        inst.__init__(*args, **kwargs)
        inst._register()
        inst._objects.extend(inst.__dict__.keys())
        return inst

class aiProcess():
    __metaclass__ = aiProcessBase
    def __init__(self, process_name):
        self._msg_observer = aiMessageObserver(self, self._ext_funcs)
        #set node name
        id = process_name[:6] if len(process_name)>6 else process_name
        self.node = cauv.node.Node("ai"+id)
        self.node.join("ai")
        self.process_name = process_name
        self.ai = aiAccess(self.node, self.process_name)
    def _register(self):
        self.node.addObserver(self._msg_observer)
        self.ai.manager.register()
    def log(self, message):
        try:
            self.node.send(messaging.AIlogMessage(message), "ai")
        except:
            error('Error sending high-level log message')
            traceback.print_exc()
    def die(self):
        info('Clearing up process %s' %(self.process_name,))
        self.node.stop()

#------AI SCRIPTS STUFF------

class fakeSonarfunction():
    def __init__(self, script, attr):
        self.script = script
        self.attr = attr
    def __call__(self, *args, **kwargs):
        self.script.ai.auv_control.sonar_command(self.attr, *args, **kwargs)
    def __getattr__(self, attr):
        error('You can only call functions of sonar, one level deep')
        raise AttributeError('fakeSonarfunction has no attribute %s' %(attr,))

class fakeSonar():
    def __init__(self, script):
        self.script = script
    def __getattr__(self, func):
        return fakeSonarfunction(self.script, func)

class fakeAUVfunction():
    def __init__(self, script, attr):
        self.script = script
        self.attr = attr
    def __call__(self, *args, **kwargs):
        debug('fakeAUVfunction: __call__ args=%s kwargs=%s' % (str(args), str(kwargs)), 5)
        self.script.ai.auv_control.auv_command(self.attr, *args, **kwargs)
    def __getattr__(self, attr):
        error('You can only call functions of AUV, one level deep (except sonar)')
        raise AttributeError('fakeAUVfunction has no attribute %s' %(attr,))

class fakeAUV(messaging.MessageObserver):
    #TODO
    #needs to respond to control overriding commands
    def __init__(self, script):
        self.script = script #passing the script saves on the number of AI_process, as fakeAUV can now call other processes through the script
        self.sonar = fakeSonar(script)
        messaging.MessageObserver.__init__(self)
        self.script.node.join("telemetry")
        self.script.node.addObserver(self)
        self.current_bearing = None
        self.current_depth = None
        self.current_pitch = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed = None
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
        
    def onLocationMessage(self, m):
        self.latitude = m.latitude
        self.longitude = m.longitude
        self.altitude = m.altitude
        self.speed = m.speed
    
    def getBearing(self):
        return self.current_bearing
    
    def bearingAndWait(self, bearing, epsilon = 5, timeout = 30):
        if bearing == None:
            self.bearing(None)
            return True
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
        if depth == None:
            self.depth(None)
            return True
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
        if pitch == None:
            self.pitch(None)
            return True
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
        debug('FakeAUV: returning dynamic override for attr=%s' % str(attr), 3)
        return fakeAUVfunction(self.script, attr)

class aiScriptOptionsBase(type):
    def __new__(cls, name, bases, attrs):
        attrs['_dynamic'] = []
        if 'Meta' in attrs:
            meta_data = attrs.pop('Meta')
            if hasattr(meta_data, 'dynamic'):
                attrs['_dynamic'] = meta_data.dynamic
                for d in attrs['_dynamic']:
                    if not d in attrs:
                        raise AttributeError('The option %s is not defined, so cannot be dynamic' %(d,))
        new_cls = super(aiScriptOptionsBase, cls).__new__(cls, name, bases, attrs)
        return new_cls
            
class aiScriptOptions():
    __metaclass__ = aiScriptOptionsBase
    def __init__(self, script_opts):
        for opt in script_opts:
            setattr(self, opt, script_opts[opt])
        
class aiScript(aiProcess):
    def __init__(self, task_name, script_opts):
        aiProcess.__init__(self, task_name)
        self.exit_confirmed = threading.Event()
        self.task_name = task_name
        self.options = script_opts
        self.auv = fakeAUV(self)
        self._pl_enabled = False
        self._pl_setup = threading.Event()
    def _register(self):
        self.node.addObserver(self._msg_observer)
    def request_pl(self, pl_name, timeout=10):
        if not self._pl_enabled:
            self.node.join('processing')
            self._pl_enabled = True
        self.ai.pipeline_manager.request_pl('script', self.task_name, pl_name)
        self._pl_setup.wait(timeout)
        if self._pl_setup.is_set():
            self._pl_setup.clear()
            return
        else:
            raise CommunicationError('No response from pipeline management.')
    def drop_pl(self, pl_name):
        self.ai.pipeline_manager.drop_pl('script', self.task_name, pl_name)
    def drop_all_pl(self):
        self.ai.pipeline_manager.drop_all_pl('script', self.task_name)
    @external_function
    def pl_response(self):
        self._pl_setup.set()
    @external_function
    def set_option(self, option_name, option_value):
        if option_name in self.options._dynamic:
            setattr(self, option_name, option_value)
            self.optionChanged(option_name)
        else:
            info('Changed the value of a static option while the script was running. Script will not see change until script restart.')
    def optionChanged(self, option_name):
        pass
    @external_function
    def depthOverridden(self):
        warning('%s tried to set a depth but was overridden and has no method to deal with this.' %(self.task_name,))
    @external_function
    def begin_override_pause(self):
        warning('AUV control by %s was paused, but this script has no methd to deal with this event' %(self.task_name,))
    @external_function
    def end_override_pause(self):
        pass
    def notify_exit(self, exit_status):
        #make sure to drop pipelines
        self.drop_all_pl()
        for x in range(5):
            self.ai.task_manager.on_script_exit(self.task_name, exit_status)
            if self.exit_confirmed.wait(1.0):
                return
        error("Task manager failed to acknowledge script "+self.task_name+" exit")
        return
    @external_function
    def confirm_exit(self):
        self.exit_confirmed.set()
    def die(self):
        self.ai.auv_control.stop()
        self.ai.auv_control.lights_off()
        aiProcess.die(self)

#------AI DETECTORS STUFF------
class aiDetectorOptions():
    def __init__(self, opts):
        for key, value in opts:
            setattr(self, key, value)
        
class aiDetector(messaging.MessageObserver):
    def __init__(self, node, opts):
        messaging.MessageObserver.__init__(self)
        self.options = opts
        self.node = node
        self.node.addObserver(self)
        self.detected = False
        self._pl_enabled = False
        self._pl_requests = {}
    def process(self):
        """
        This should define a method to do any intensive (ie not on message) processing
        """
        pass
    def request_pl(self, pl_name):
        if pl_name in self._pl_requests:
            self._pl_requests[pl_name] += 1
        else: self._pl_requests[pl_name] = 1
    def drop_pl(self, pl_name):
        if pl_name in self._pl_requests:
            if self._pl_requests[pl_name] > 0:
                self._pl_requests[pl_name] -= 1
                return
        error("Can't drop pipeline that hasn't been requested")
    def drop_all_pl(self):
        self._pl_requests = {}
    def log(self, message):
        try:
            self.node.send(messaging.AIlogMessage(message), "ai")
        except:
            error('Error sending high-level log message')
            traceback.print_exc()
    def die(self):
        self.drop_all_pl()
        self.node.removeObserver(self)
    def optionChanged(self, option_name):
        pass

#------AI TASKS STUFF------

class aiCondition():
    """
    Basic condition that can be used for tasks, ie they may be set via task_manager from any other process
    and then when they change task manager checks to see if the conditions for any task have been met
    """
    def __init__(self, name, state=False):
        #defines what values are needed to re-init the condition when unpickled
        self.store = ['name', 'state']
        self.name = name
        self.state = state
        self.state_lock = threading.Lock()
    #pickling stuff
    def __getstate__(self):
        #since bits and pieces need to be setup for a condition we will call init instead of restoring __dict__
        return dict([(x,getattr(self, x)) for x in self.store])
    def __setstate__(self, state):
        #restore values
        self.__init__(**state)
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
    def __init__(self, name, default_time=0, state=False):
        self.store = ['name', 'default_time']
        self.name = name
        self.default_time = default_time
        self.state_lock = threading.Lock()
        self.timeout = time.time()+default_time if state else None
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
        
class timeoutCondition(timeCondition):
    def __init__(self, name, default_time=30, state=False):
        timeCondition.__init__(self, name, default_time, not state)
    def get_state(self):
        return not timeCondition.get_state(self)
    
class detectorCondition(aiCondition):
    """
    This condition relies on the state of a detector
    """
    def __init__(self, name, detector_name, state=False):
        aiCondition.__init__(self, name, state)
        self.store.append('detector_name')
        self.detector_name = detector_name
    def register(self, task_manager):
        aiCondition.register(self, task_manager)
        #We need to tell the task manager to setup the detector, and redirect messages to this condition
        task_manager.add_detector(self.detector_name, self)
    def deregister(self, task_manager):
        task_manager.remove_detector(self.detector_name, self)
        aiCondition.deregister(self, task_manager)

class aiTask():
    def __init__(self, name, script_name, priority, running_priority=None, detectors_enabled=False, conditions=None, crash_limit=5, frequency_limit=30, options=None, **kwargs):
        """
        Defines a 'Task', a script to run and when to run it
        Options:
        -name, str, name of task
        -script_name, str, the filename of the script (minus '.py')
        -priority, int, the priority of the script, larger numbers = higher priority
        -running_priority, int, the priority of the script once it has started (default to priority)
        -detectors_enabled, bool, whether to keep the detectors running while the script is running (default false)
        -conditions, [aicondition,] a list of conditions for the script to be run
        -options, a dictionary of arg, value pairs to be passed to the script
        Note: any left over keyword arguments are appened to options
        The default is_available method waits till all conditions are True, this can be changed by redefining is_available()
        """
        self.name = name
        self.script_name = script_name
        self.conditions = conditions if conditions else []
        self.priority = priority
        self.running_priority = running_priority if running_priority else priority
        self.detectors_enabled = detectors_enabled
        self.options = options if options else {}
        self.options.update(kwargs)
        self.registered = False
        self.active = False
        self.crash_count = 0
        self.crash_limit = crash_limit
        self.frequency_limit = frequency_limit# once every
        self.last_called = 0
    def update_options(self, options={}, **kwargs):
        """
        Updates the options on the task, accepts dict or kwargs
        """
        self.options.update(options)
        self.options.update(kwargs)
    def register(self, task_manager):
        if self.registered:
            error('Task already setup')
            return
        task_manager.active_tasks.append(self)
        for condition in self.conditions:
            condition.register(task_manager)
        self.registered = True
    def deregister(self, task_manager):
        if not self.registered:
            error('Task not setup, so can not be deregistered')
            return
        task_manager.active_tasks.remove(self)
        for condition in self.conditions:
            condition.deregister(task_manager)
        self.registered = False
    def is_available(self):
        for condition in self.conditions:
            if not condition.get_state():
                return False
        return True
