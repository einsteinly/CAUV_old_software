import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import threading
import cPickle
import time
import traceback
import collections
import utils.event as event
import inspect

debug_converters = {threading._Event: lambda x:x.is_set(),
                    }

#------AI PROCESSES STUFF------
FuncCall = collections.namedtuple("FuncCall", ['process', 'calling_process', 'function', 'args', 'kargs'])

class CommunicationError(Exception):
    pass

class aiForeignFunction():
    def __init__(self, node, calling_process, process, function):
        self.node = node
        self.calling_process = calling_process
        self.process = process
        self.function = function
    def __call__(self, *args, **kwargs):
        message = cPickle.dumps(FuncCall(self.process, self.calling_process, self.function, args, kwargs))
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
        
ExternalFunction = object()

def external_function(f):
    """Mark a function as external and thus callable from other processes via
    aiAccess (usually self.ai.process_name.external_function())"""
    f.external = ExternalFunction
    if not hasattr(f, 'caller'):
        try:
            f.caller = 'calling_process' in f.func_code.co_varnames
        except AttributeError:
            f.caller = False
    return f

def force_calling_process(f):
    f.caller = True
    return f

class aiProcess(event.EventLoop, messaging.MessageObserver):
    def __init__(self, process_name):
        #Need to be careful with use of super(), since there's now multiple
        #inheritance. It can act unexpectedly (for instance, switching the order
        #of inheritance of aiProcess will break things currently)
        super(aiProcess, self).__init__()
        #set node name
        #id = process_name[:6] if len(process_name)>6 else process_name
        self.node = cauv.node.Node("ai_"+process_name)
        self.node.join("ai")
        self.node.join("guiai")
        self.process_name = process_name
        self.ai = aiAccess(self.node, self.process_name)
        self.running = True
        self.external_functions = {}
        for name, member in inspect.getmembers(self):
            try:
                if member.external is ExternalFunction:
                    self.external_functions[name] = member
            except AttributeError:
                pass
        print(self.external_functions)
        self.node.addObserver(self)
        self.ai.manager.register()

    def onAIMessage(self, m):
        message = cPickle.loads(m.msg)
        debug("onAIMessage in %s: %s" %(self.process_name, message), 4)
        if message.process == self.process_name:
            try:
                func = self.external_functions[message.function]
                if func.caller:
                    message.kargs['calling_process'] = message.calling_process
                try:
                    func(*message.args, **message.kargs)
                except Exception as e:
                    error(traceback.format_exc().encode('ascii', 'replace'))
            except KeyError:
                error("Unknown function {} called by {}".format(message.function, message.calling_process))

    def log(self, message):
        debug(message)
        try:
            self.node.send(messaging.AIlogMessage(message), "ai")
        except:
            error('Error sending high-level log message')
            error(traceback.format_exc().encode('ascii', 'replace'))

    def die(self):
        self.running = False
        info('Clearing up process %s' %(self.process_name,))
        self.node.stop()

#------AI OPTIONS STUFF------

class aiOptionsBase(type):
    def __new__(cls, name, bases, attrs):
        new_attrs = {'_option_classes':{}}
        #add in 'inherited' options from parent class
        for base in bases:
            for key, attr in base.__dict__.iteritems():
                #but don't overwrite values
                if not key in attrs:
                    attrs[key] = attr
        for key, value in attrs.iteritems():
            #don't process 'system' values
            if not key[0] == '_':
                #if user defines a type for the option, force the type, and store type in meta (assume that this type is transmittable)
                if isinstance(value, tuple) and len(value)==2 and callable(value[1]):
                    new_attrs[key] = value[1](value[0])
                    new_attrs['_option_classes'][key]=value[1]
                #else leave (if can be transmitted)
                elif isinstance(value, (int,str,float,bool)):
                    new_attrs[key] = value
                #else need to make sure don't try to transmit, so relabel
                else:
                    new_attrs['_not_transmittable_'+key] = value
                    if not callable(value):
                        warning('Option %s on %s will not appear as is not a valid type' %(key, name))
            else:
                new_attrs[key] = value
        new_cls = super(aiOptionsBase, cls).__new__(cls, name, bases, new_attrs)
        return new_cls
    def __getattr__(cls, attr):
        #make sure also searches _not_transmittable stuff
        return cls.__getattribute__(cls, '_not_transmittable_'+attr)
    def get_default_options(cls):
        return dict([item for item in cls.__dict__.iteritems() if item[0][0] != '_'])
    def get_default_options_as_params(cls):
        #make sure converted to params if needed
        options={}
        for key, attr in cls.__dict__.iteritems():
            if key in cls._option_classes and hasattr(cls._option_classes[key], 'asParamValue'):
                options[key] = cls._option_classes[key].asParamValue(attr)
            else: options[key] = attr
        return options
    
class aiOptions(object):
    __metaclass__ = aiOptionsBase
    def __init__(self, options={}):
        #set values to default
        self.__dict__.update(self.__class__.get_default_options()) #pylint: disable=E1101
        for opt, val in options.iteritems():
            setattr(self, opt, val)
    def __getattr__(self, attr):
        return self.__getattribute__('_not_transmittable_'+attr)
    def __setattr__(self, key, attr):
        #force type if specified in meta
        if key in self._option_classes:
            attr=self._option_classes[key](attr)
        object.__setattr__(self, key, attr)
    def get_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_'])
    def get_options_as_params(self):
        #make sure converted to params if needed
        options={}
        for key, attr in self.__dict__.iteritems():
            if key in self._option_classes and hasattr(self._option_classes[key], 'asParamValue'):
                options[key] = self._option_classes[key].asParamValue(attr)
            else: options[key] = attr
        return options
        
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
        self.lla = None
        #self.altitude = None
        #self.speed = None
        self.bearingCV = threading.Condition()
        self.depthCV = threading.Condition()
        self.pitchCV = threading.Condition()
        
    def onTelemetryMessage(self, m):
        #self.bearing = m.orientation.yaw
        self.current_bearing = m.orientation.yaw
        self.current_depth = m.depth
        self.current_pitch = m.orientation.pitch
        with self.bearingCV:
            with self.depthCV:
                with self.pitchCV:
                    self.bearingCV.notifyAll()
                    self.depthCV.notifyAll()
                    self.pitchCV.notifyAll()
        
    #def onLocationMessage(self, m):
    #self.latitude = m.location.latitude
    #self.longitude = m.location.longitude
    #self.altitude = m.location.altitude
    #self.speed = m.speed
    
    def getBearing(self):
        return self.current_bearing
    
    def bearingAndWait(self, bearing, epsilon = 5, timeout = 20):
        if bearing == None:
            self.bearing(None)
            return True
        startTime = time.time()
        self.bearing(bearing)
        while time.time() - startTime < timeout:
            if self.current_bearing == None or min((bearing - self.current_bearing) % 360, (self.current_bearing - bearing) % 360) > epsilon:
                #print 'bearing waiting'
                with self.bearingCV:
                    self.bearingCV.wait(timeout - time.time() + startTime)
            else:
                return True
        return False
        
    def depthAndWait(self, depth, epsilon = 5, timeout = 20):
        if depth == None:
            self.depth(None)
            return True
        startTime = time.time()
        self.depth(depth)
        while time.time() - startTime < timeout:
            if self.current_depth == None or abs(depth - self.current_depth) > epsilon:
                with self.depthCV:
                    self.depthCV.wait(timeout - time.time() + startTime)
            else:
                return True
        return False

    def pitchAndWait(self, pitch, epsilon = 5, timeout = 20):
        if pitch == None:
            self.pitch(None)
            return True
        startTime = time.time()
        self.pitch(pitch)
        while time.time() - startTime < timeout:
            if self.current_pitch == None or min((pitch - self.current_pitch) % 360, (self.current_pitch - pitch) % 360) > epsilon:
                with self.pitchCV:
                    self.pitchCV.wait(timeout - time.time() + startTime)
            else:
                return True
        return False
        
    def __getattr__(self, attr):
        debug('FakeAUV: returning dynamic override for attr=%s' % str(attr), 3)
        return fakeAUVfunction(self.script, attr)

class aiScriptOptionsBase(aiOptionsBase):
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
    def __getattr__(cls, attr):
        return cls.__getattribute__(cls, '_not_transmittable_'+attr)
    def get_default_options(cls):
        return dict([item for item in cls.__dict__.iteritems() if item[0][0] != '_'])
    def get_dynamic_options(cls):
        return dict([item for item in cls.__dict__.iteritems() if item[0][0] != '_' and item[0] in cls._dynamic])
    def get_static_options(cls):
        return dict([item for item in cls.__dict__.iteritems() if item[0][0] != '_' and not item[0] in cls._dynamic])
            
class aiScriptOptions(aiOptions):
    __metaclass__ = aiScriptOptionsBase
    reporting_frequency = 2
    def get_dynamic_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_' and item[0] in self._dynamic])
    def get_static_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_' and (not item[0] in self._dynamic)])
    def get_dynamic_options_as_params(self):
        return dict([(key, self._option_classes[key](attr) if key in self._option_classes else attr) for key, attr in self.__dict__.iteritems() if key[0] != '_' and key in self._dynamic])
    def get_static_options_as_params(self):
        return dict([(key, self._option_classes[key](attr) if key in self._option_classes else attr) for key, attr in self.__dict__.iteritems() if key[0] != '_' and (not key in self._dynamic)])
        
class aiScriptState(object):
    def __init__(self, state):
        for key, val in state.items():
            object.__setattr__(self, key, val)
    def own(self, parent_script):
        self._parent_script = parent_script
    def __setattr__(self, key, attr):
        object.__setattr__(self, key, attr)
        if not key[0] == '_':
            self._parent_script.ai.task_manager.on_persist_state_change(self._parent_script.task_name, key, attr)
        
class aiScript(aiProcess):
    debug_values = []
    def __init__(self, task_name, script_opts, persistent_state):
        aiProcess.__init__(self, task_name)
        self.die_flag = threading.Event() #for any subthreads
        self.exit_confirmed = threading.Event()
        self.in_control = threading.Event()
        self.pl_confirmed = threading.Event()
        self._requested_pls = []
        self.task_name = task_name
        self.options = script_opts
        self.auv = fakeAUV(self)
        self.persist = persistent_state
        #take ownership to ensure that changes get directed back
        self.persist.own(self)
        self.reporting_thread=threading.Thread(target=self.report_loop)
        self.reporting_thread.start()
    def _register(self):
        self.node.addObserver(self._msg_observer)
    #image pipeline stuff
    def request_pl(self, pl_name, timeout=10):
        self.pl_confirmed.clear()
        self.ai.pl_manager.request_pl('script', self.task_name, pl_name)
        self._requested_pls.append('ai/'+pl_name)
        return self.pl_confirmed.wait(timeout)
    def drop_pl(self, pl_name):
        self.ai.pl_manager.drop_pl('script', self.task_name, pl_name)
        try:
            self._requested_pls.remove('ai/'+pl_name)
        except ValueError:
            error("Can't remove a pipeline that hasn't been requested")
    def drop_all_pl(self):
        self.ai.pl_manager.drop_all_pls('script', self.task_name)
        self._requested_pls = []
    @external_function
    def confirm_pl_request(self):
        self.pl_confirmed.set()
    #option stuff
    @external_function
    def set_option(self, option_name, option_value):
        if option_name in self.options._dynamic:
            setattr(self.options, option_name, option_value)
            self.optionChanged(option_name)
        else:
            info('Changed the value of a static option while the script was running. Script will not see change until script restart.')
    @external_function
    def set_options(self, options):
        for key, val in options.items():
            self.set_option(key,val)
    def optionChanged(self, option_name):
        pass
    #control stuff
    @external_function
    def _set_position(self, llacoord):
        self.auv.lla = llacoord
    def request_control(self, timeout=None):
        self.ai.auv_control.request_control(timeout)
    def request_control_and_wait(self, wait_timeout=5, control_timeout=None):
        self.ai.auv_control.request_control(control_timeout)
        return self.in_control.wait(wait_timeout)
    def drop_control(self):
        self.ai.auv_control.drop_control()
    @external_function
    def depthOverridden(self):
        warning('%s tried to set a depth but was overridden and has no method to deal with this.' %(self.task_name,))
    #note that _ functions are called by auv control, to make sure things like waiting for control are dealt with properly
    def set_paused(self):
        warning('AUV control by %s was paused, but this script has no method to deal with this event' %(self.task_name,))
    def set_unpaused(self):
        warning('AUV control by %s was unpaused, but this script has no method to deal with this event' %(self.task_name,))
    @external_function
    def _set_paused(self):
        self.in_control.clear()
        self.set_paused()
    @external_function
    def _set_unpaused(self):
        self.in_control.set()
        self.set_unpaused()
    @external_function
    def control_timed_out(self):
        warning('AUV control by %s timed out, but this script has no method to deal with this event' %(self.task_name,))
    #debug value reporting etc
    def report_status(self):
        debug = {}
        error_attrs = []
        for key_str in self.debug_values:
            keys = key_str.split('.')
            value = self
            try:
                for key in keys:
                    value = getattr(value, key)
                #make sure that we can transmit it
                try:
                    value = messaging.ParamValue.create(value)
                except TypeError:
                    value = messaging.ParamValue.create(debug_converters[type(value)](value))
            except Exception:
                error_attrs.append(key_str)
                continue
            debug[key_str] = value
        if error_attrs:
            warning("Could not get/encode attributes %s, skipping from debug value report" %str(error_attrs))
        try:
            self.node.send(messaging.ScriptStateMessage(self.task_name,debug,self._requested_pls))
        except Exception as e:
            traceback.print_exc()
    def report_loop(self):
        while not self.die_flag.wait(self.options.reporting_frequency):
            self.report_status()
        debug("Exiting reporting thread.")
    def _notify_exit(self, exit_status):
        #make sure to drop pipelines
        self.drop_all_pl()
        self.ai.task_manager.on_script_exit(self.task_name, exit_status)
        if self.exit_confirmed.wait(5.0):
            return
        error("Task manager failed to acknowledge script "+self.task_name+" exit")
        return
    @external_function
    def confirm_exit(self):
        self.exit_confirmed.set()
    def die(self):
        self.die_flag.set()
        self.ai.auv_control.stop()
        self.ai.auv_control.lights_off()
        aiProcess.die(self)
        
#------AI DETECTORS STUFF------        
class aiDetectorOptions(aiOptions):
    pass
        
class aiDetector(messaging.MessageObserver):
    debug_values = []
    def __init__(self, node, opts):
        messaging.MessageObserver.__init__(self)
        self._pipelines = []
        self.options = opts
        self.node = node
        self.node.addObserver(self)
        self.detected = False
    def request_pl(self, name):
        self._pipelines.append(name)
    def drop_pl(self, name):
        self._pipelines.remove(name)
    def drop_all_pl(self):
        self._pipelines = []
    def process(self):
        """
        This should define a method to do any intensive (ie not on message) processing
        """
        pass
    def set_option(self, option_name, option_value):
        setattr(self.options, option_name, option_value)
        self.optionChanged(option_name)
    def get_debug_values(self):
        debug = {}
        error_attrs = []
        for key_str in self.debug_values:
            keys = key_str.split('.')
            value = self
            try:
                for key in keys:
                    value = getattr(value, key)
                #make sure that we can transmit it
                try:
                    value = messaging.ParamValue.create(value)
                except TypeError:
                    value = messaging.ParamValue.create(debug_converters[type(value)](value))
            except Exception:
                error_attrs.append(key_str)
                continue
            debug[key_str] = value
        if error_attrs:
            warning("Could not get/encode attributes %s, skipping from debug value report" %str(error_attrs))
        return debug
    def log(self, message):
        debug(message)
        try:
            self.node.send(messaging.AIlogMessage(message), "ai")
        except:
            error('Error sending high-level log message')
            error(traceback.format_exc().encode('ascii','ignore'))
    def die(self):
        self.drop_all_pl()
        self.node.removeObserver(self)
    def optionChanged(self, option_name):
        pass
    
#------GENERAL STUFF------
#this should probably be in utils

def subclassDict(cls_with_subs):
    classes = {}
    try:
        to_check = set(cls_with_subs.__subclasses__())
    except AttributeError:
        raise TypeError('Class must ultimately derive from object (new-style classes) not old style classes')
    checked = set()
    while len(to_check):
        cur = to_check.pop()
        checked.add(cur)
        if not getattr(cur, '_abstract', False):
            classes[cur.__name__] = cur
        for sub in cur.__subclasses__():
            if not sub in checked:
                to_check.add(sub)
    return classes

class RepeatTimer(threading.Thread):
    def __init__(self, time, function, args=[], kwargs={}):
        threading.Thread.__init__(self)
        self.time = time
        self.func = function
        self.args = args
        self.kwargs = kwargs
        self.die_flag = threading.Event()
    def run(self):
        while not self.die_flag.wait(self.time):
            self.func(*self.args, **self.kwargs)
        debug('Cleared up periodic timer.')
