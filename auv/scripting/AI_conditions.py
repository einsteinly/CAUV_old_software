import detector_library
import threading

from AI_classes import subclassDict

class conditionOptions(object):
    def __init__(self, options={}):
        for key, attr in self.__class__.__dict__.iteritems():
            if key[0] != '_':
                setattr(self, key, attr)
        self.__dict__.update(options)
    def get_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_'])

class aiCondition(object):
    class options():
        pass
    def __new__(cls, *args, **kwargs):
        inst = super(aiCondition, cls).__new__(cls, *args, **kwargs)
        #make options an instance
        inst.options = cls.options()
        return inst
    def __init__(self, options=None):
        if options:
            self.set_options(options)
    def set_options(self, options):
        for name, value in options.iteritems():
            setattr(self.options, name, value)
    def register(self, task_manager):
        self.id, self.change_event = task_manager.add_condition(self)
    def deregister(self, task_manager):
        task_manager.remove_condition(self.id)
            
class stateCondition(aiCondition):
    """
    Basic condition that just has a settable state
    """
    class options():
        state = False
    def set_options(self, options):
        try:
            if options['state'] != self.options.state:
                self.options.state = options['state']
                self.change_event.set()
        except KeyError:
            pass
    def get_state(self):
        return self.options['state']
        
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
    anything with this metaclass will become a none!!!
    """
    def __init__(cls, name, bases, attrs):
        #basically we want to create a whole load of new classes based on this one and some data from the detector library
        if attrs.pop('_abstract', False):
            for detector_name in detector_library.__all__:
                attrs['_abstract'] = False
                attrs['detector_name'] = detector_name
                type(detector_name+'Condition', (cls, ), attrs)
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
        opts = __import__('detector_library.'+cls.detector_name, fromlist=['detectorOptions']).detectorOptions.get_default_options()
        cls.options = type('options', (conditionOptions, ), opts)
        return super(detectorConditions, cls).__new__(cls, *args, **kwargs)
    def __init__(self, options={}):
        aiCondition.__init__(self, options)
        self.state = False
        self.detector = None
    def set_options(self, options):
        self.detector.set_options(options)
    def on_options_set(self, options):
        for name, value in options.iteritems():
            setattr(self.options, name, value)
    def on_state_set(self, state):
        if state != self.state:
            self.state = state
            self.change_event.set()
    def register(self, task_manager):
        aiCondition.register(self, task_manager)
        #We need to tell the task manager to setup the detector, and redirect messages to this condition
        self.detector = task_manager.add_detector(self.detector_name, self)
    def deregister(self, task_manager):
        task_manager.remove_detector(self.detector_name, self)
        aiCondition.deregister(self, task_manager)


conditions = subclassDict(aiCondition)