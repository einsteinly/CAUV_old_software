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
        setattr(self, function, aiForeignFunction(self.node, self.calling_process, self.process, function))
        return getattr(self, function)
        
class aiAccess():
    def __init__(self, node, process_name):
        self.node = node
        self.process_name = process_name
    def __getattr__(self, process):
        setattr(self, process, aiForeignProcess(self.node, self.process_name, process))
        return getattr(self, process)
        
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

def onMessageFactory(self, m_func_name):
    def onMessageFunction(m):
        return getattr(self.parent_process, m_func_name)(m)
    return onMessageFunction

class aiMessageObserver(messaging.MessageObserver):
    key_index = {'to': 0, 'from': 1, 'function': 2, 'args': 3, 'kwargs': 4}
    def __init__(self, parent_process):
        messaging.MessageObserver.__init__(self)
        self.parent_process = parent_process
        for on_message in self.parent_process._on_messages:
            setattr(self, on_message, onMessageFactory(self, on_message))
    def on_message(self, m_func, m):
        return m_func(self.parent_process, m)
    def onAIMessage(self, m):
        debug("onAIMessage in %s: %s" %(self.parent_process.process_name, m.msg), 6)
        message = cPickle.loads(m.msg)
        if message[0] == self.parent_process.process_name: #this is where the to string appears in the cpickle output
            message = cPickle.loads(m.msg)
            if message[2] in self.parent_process._ext_funcs:
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
        if not hasattr(cls, '_on_messages'):
            cls._on_messages = []
        for key, attr in attrs.iteritems():
            if isinstance(attr, external_function):
                cls._ext_funcs.append(key)
                setattr(cls, key, attr.func)
            if key[:2] == 'on' and key [-7:] == 'Message':
                cls._on_messages.append(key)
    def __call__(cls, *args, **kwargs):
        inst = cls.__new__(cls, *args, **kwargs)
        inst.__init__(*args, **kwargs)
        inst._register()
        return inst

class aiProcess():
    __metaclass__ = aiProcessBase
    def __init__(self, process_name):
        self._msg_observer = aiMessageObserver(self)
        #set node name
        id = process_name[:6] if len(process_name)>6 else process_name
        self.node = cauv.node.Node("ai"+id)
        self.node.join("ai")
        self.node.join("guiai")
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
        attrs2 = {}
        for key, value in attrs.iteritems():
            if not key[0] == '_':
                if isinstance(value, (int,str,float,bool)):
                    attrs2[key] = value
                else:
                    attrs2['_not_transmittable_'+key] = value
                    warning('Option %s will not appear as is not a valid type' %key)
            else:
                attrs2[key] = value
        new_cls = super(aiScriptOptionsBase, cls).__new__(cls, name, bases, attrs2)
        return new_cls
    def __getattr__(cls, attr):
        return cls.__getattribute__(cls, '_not_transmittable_'+attr)
    def get_default_options(cls):
        return dict([item for item in cls.__dict__.iteritems() if item[0][0] != '_'])
    def get_dynamic_options(cls):
        return dict([item for item in cls.__dict__.iteritems() if item[0][0] != '_' and item[0] in cls._dynamic])
    def get_static_options(cls):
        return dict([item for item in cls.__dict__.iteritems() if item[0][0] != '_' and not item[0] in cls._dynamic])
            
class aiScriptOptions():
    __metaclass__ = aiScriptOptionsBase
    def __init__(self, script_opts):
        for opt in script_opts:
            setattr(self, opt, script_opts[opt])
    def __getattr__(self, attr):
        return self.__getattribute__(self, '_not_transmittable_'+attr)
    def get_default_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_'])
    def get_dynamic_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_' and item[0] in self._dynamic])
    def get_static_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_' and not item[0] in self._dynamic])
        
class aiScript(aiProcess):
    def __init__(self, task_name, script_opts):
        aiProcess.__init__(self, task_name)
        self.exit_confirmed = threading.Event()
        self.task_name = task_name
        self.options = script_opts
        self.auv = fakeAUV(self)
    def _register(self):
        self.node.addObserver(self._msg_observer)
    def request_pl(self, pl_name, timeout=10):
        pass
    def drop_pl(self, pl_name):
        pass
    def drop_all_pl(self):
        pass
    @external_function
    def set_option(self, option_name, option_value):
        if option_name in self.options._dynamic:
            setattr(self.options, option_name, option_value)
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
class aiDetectorOptionsBase(type):
    def __new__(cls, name, bases, attrs):
        attrs2 = {}
        for key, value in attrs.iteritems():
            if not key[0] == '_':
                if isinstance(value, (int,str,float,bool)):
                    attrs2[key] = value
                else:
                    attrs2['_not_transmittable_'+key] = value
                    warning('Option %s will not appear as is not a valid type' %key)
            else:
                attrs2[key] = value
        new_cls = super(aiDetectorOptionsBase, cls).__new__(cls, name, bases, attrs2)
        return new_cls
    def get_default_options(cls):
        return dict([item for item in cls.__dict__.iteritems() if item[0][0] != '_'])
        
class aiDetectorOptions(object):
    __metaclass__ = aiDetectorOptionsBase
    def __init__(self, options={}):
        for key, value in options:
            setattr(self, key, value)
    def get_default_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_'])
    def __getattr__(self, attr):
        return getattr(self, '_not_transmittable_'+attr)
        
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
    def set_option(self, option_name, option_value):
        setattr(self.options, option_name, option_value)
        self.optionChanged(option_name)
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
    
#------GENERAL STUFF------

class subclassDict(object):
    def __init__(self, cls_with_subs):
        self.classes = {}
        try:
            to_check = set(cls_with_subs.__subclasses__())
        except AttributeError:
            raise TypeError('Class must ultimately derive from object (new-style classes) not old style classes')
        checked = set()
        while len(to_check):
            cur = to_check.pop()
            checked.add(cur)
            if not getattr(cur, '_abstract', False):
                self.classes[cur.__name__] = cur
            for sub in cur.__subclasses__():
                if not sub in checked:
                    to_check.add(sub)
        print "available classes", self.classes
    def __getitem__(self, attr):
        return self.classes[attr]
    def __getattr__(self, attr):
        return self.classes[attr]

class RepeatTimer(threading.Thread):
    def __init__(self, time, function, args=[], kwargs={}):
        threading.Thread.__init__(self)
        self.time = time
        self.func = function
        self.args = args
        self.kwargs = kwargs
        self.die = False
    def run(self):
        while True:
            time.sleep(self.time)
            self.func(*self.args, **self.kwargs)
            if self.die:
                break
