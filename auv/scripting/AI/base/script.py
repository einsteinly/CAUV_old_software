import threading

from cauv import messaging

from AI.base.fake_auv import fakeAUV
from AI.base.option import aiOptions, aiOptionsBase
from AI.base.proc import aiProcess, external_function

debug_converters = {threading._Event: lambda x:x.is_set(),
                    }
                    
class aiScriptOptionsBase(aiOptionsBase):
    def __new__(cls, name, bases, attrs):
        attrs['_dynamic'] = []
        if 'Meta' in attrs:
            meta_data = attrs['Meta']
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
        self.pl_confirmed = threading.Event()
        self.task_name = task_name
        self.options = script_opts
        self.auv = fakeAUV(self)
        self.persist = persistent_state
        #take ownership to ensure that changes get directed back
        self.persist.own(self)
        self._last_log = ''
        self.reporting_thread=threading.Thread(target=self.report_loop)
        self.reporting_thread.start()
    #override log to store value
    def log(self, message):
        aiProcess.log(self, message)
        self._last_log = message
        
    #image pipeline stuff
    def request_pl(self, pl_name, timeout=10):
        raise NotImplementedError("This feature has been removed, pipelines should be requested by placing them in the requested pipelines list")
    def drop_pl(self, pl_name):
        raise NotImplementedError("Removed for new pipeline management system.")
    def drop_all_pl(self):
        raise NotImplementedError("Removed for new pipeline management system.")
    #option stuff
    @external_function
    def set_option(self, option_name, option_value):
        if option_name in self.options._dynamic:
            setattr(self.options, option_name, option_value)
            self.optionChanged(option_name)
        else:
            info('Changed the value of a static option %s while the script was running. Script will not see change until script restart.' %option_name)
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
    @external_function
    def depthOverridden(self):
        warning('%s tried to set a depth but was overridden and has no method to deal with this.' %(self.task_name,))#debug value reporting etc
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
        debug['last_log_message'] = self._last_log
        if error_attrs:
            warning("Could not get/encode attributes %s, skipping from debug value report" %str(error_attrs))
        try:
            self.node.send(messaging.ScriptStateMessage(self.task_name,debug))
        except Exception as e:
            traceback.print_exc()
    def report_loop(self):
        while not self.die_flag.wait(self.options.reporting_frequency):
            self.report_status()
        debug("Exiting reporting thread.")
    def _notify_exit(self, exit_status):
        #make sure to drop pipelines
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