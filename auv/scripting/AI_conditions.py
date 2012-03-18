import detector_library
import threading

from cauv.debug import debug, warning, error, info

from AI_classes import subclassDict

class conditionOptions(object):
    def __init__(self, options={}):
        #add default options defined in subclasses
        for key, attr in self.__class__.__dict__.iteritems():
            if key[0] != '_':
                setattr(self, key, attr)
        #overide default sith value set by init
        self.__dict__.update(options)
    def get_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_'])

class aiCondition(object):
    """
    base condition
    """
    #make sure there is definately an options class
    class options(conditionOptions):
        pass
    def __new__(cls, *args, **kwargs):
        inst = super(aiCondition, cls).__new__(cls, *args, **kwargs)
        #make options an instance
        inst.options = cls.options()
        return inst
    def __init__(self, options=None):
        if options:
            self.set_options(options)
        self.task_ids = []
    def set_options(self, options):
        for name, value in options.items():
            setattr(self.options, name, value)
    def register(self, task_manager):
        self.id  = task_manager.register_condition(self)
    def deregister(self, task_manager):
        for task_id in self.task_ids:
            debug('removing condition %d from task %d' %(self.id, task_id), 5)
            task_manager.tasks[task_id].conditions.pop(self.id)
    def get_options(self):
        return self.options.get_options()
    def get_debug_values(self):
        warning('Debug values not implemented in condition %s' %str(self.__class__))
        return {}
    @staticmethod
    def get_pipeline_names():
        warning('Pipeline dependancies not implemented in condition')
        return []
            
class stateCondition(aiCondition):
    """
    Basic condition that just has a settable state
    """
    class options(conditionOptions):
        state = False
    def set_options(self, options):
        try:
            if options['state'] != self.options.state:
                self.options.state = options['state']
                if hasattr(self, 'change_event'):
                    self.change_event.set()
        except KeyError:
            pass
    def get_state(self):
        return self.options.state
        
class timeCondition(stateCondition):
    """
    This condition only remains true for a certain time
    """
    class options:
        timeout = 30
        startTimer = False
    def __init__(self, options={}, initial_state=False):
        stateCondition.__init__(self, options)
        self.timer = None
        self.state = initial_state
    def set_options(self, options):
        start = options.pop('startTimer', False)
        stateCondition.set_options(self, options)
        if start:
            if self.timer:
                self.timer.cancel()
            self.timer = threading.Timer(self.options.timeout, self.timeout)
            self.state = True
            self.timer.start()
            self.change_event.set()
    def timeout(self):
        self.state = False
        self.change_event.set()
        self.timer = None
    def get_state(self):
        return self.state
        
class timeoutCondition(timeCondition):
    def __init__(self, options={}):
        timeCondition.__init__(self, options, initial_state=True)
    def get_state(self):
        return not timeCondition.get_state(self)

class detectorConditionBase(type):
    """
    anything with this metaclass will become a None (but the subclasses will be created)!!!
    """
    def __init__(cls, name, bases, attrs):
        #basically we want to create a whole load of new classes based on this one and some data from the detector library
        list_of_subclasses = []
        if attrs.pop('_abstract', False):
            for detector_name in detector_library.__all__:
                attrs['_abstract'] = False
                attrs['detector_name'] = detector_name
                list_of_subclasses.append(type(detector_name+'Condition', (cls, ), attrs))
        attrs['_subclass_list_do_not_edit_please_this_is_here_just_to_keep_references'] = list_of_subclasses
        return super(detectorConditionBase, cls).__init__(name, bases, attrs)

class detectorConditions(aiCondition):
    __metaclass__=detectorConditionBase
    _abstract = True
    """
    This condition represents a detector
    All the options here are duplicated in the detector
    """
    def __new__(cls, *args, **kwargs):
        #need to get default options
        module = __import__('detector_library.'+cls.detector_name, fromlist=['detectorOptions'])
        opts = module.detectorOptions.get_default_options()
        cls.pipelines = module.detector.pipelines
        cls.options = type('options', (conditionOptions, ), opts)
        return super(detectorConditions, cls).__new__(cls, *args, **kwargs)
    def __init__(self, options={}):
        aiCondition.__init__(self, options)
        self.state = False
        self.detector = None
    def set_options(self, options):
        self.task_manager.set_detector_options(self.detector_id, self.options.get_options())
    def on_state_set(self, state):
        if state != self.state:
            self.state = state
    def register(self, task_manager):
        aiCondition.register(self, task_manager)
        #We need to tell the task manager to setup the detector, and redirect messages to this condition
        self.detector_id = task_manager.add_detector(self.detector_name, self)
        task_manager.set_detector_options(self.detector_id, self.options.get_options())
        self.task_manager = task_manager
    def deregister(self, task_manager):
        task_manager.remove_detector(self.detector_id)
        aiCondition.deregister(self, task_manager)
    def get_state(self):
        return self.state


conditions = subclassDict(aiCondition)
