import sys
import threading
import detector_library

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
        self._suppress_reporting = False
    def set_options(self, options):
        for name, value in options.items():
            setattr(self.options, name, value)
    def register(self, task_manager):
        self.id  = task_manager.register_condition(self)
    def deregister(self, task_manager):
        for task_id in self.task_ids:
            debug('removing condition %s from task %s' %(self.id, task_id), 5)
            task_manager.tasks[task_id].conditions.pop(self.id)
    def get_options(self):
        return self.options.get_options()
    def get_debug_values(self):
        #warning('Debug values not implemented in condition %s' %str(self.__class__))
        return {}
    def get_pipeline_ids(self):
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
        except KeyError:
            pass
    def get_state(self):
        return self.options.state
        
class timeCondition(aiCondition):
    """
    This condition only remains true for a certain time
    """
    class options(conditionOptions):
        timeout = 30
        startTimer = False
    def __init__(self, options={}, initial_state=False):
        aiCondition.__init__(self, options)
        self.timer = None
        self.timer_started = False
        self.state = initial_state
    def set_options(self, options):
        start = options.pop('startTimer', False)
        aiCondition.set_options(self, options)
        if start:
            if self.timer:
                self.timer.cancel()
            self.timer = threading.Timer(self.options.timeout, self.timeout) # pylint: disable=E1101
            self.timer_started = True
            self.state = True
            self.timer.start()
    def get_debug_values(self):
        return {'Timer Started': self.timer_started}
    def timeout(self):
        self.state = False
        self.timer_started = False
        self.timer = None
    def get_state(self):
        return self.state
        
class timeoutCondition(timeCondition):
    def __init__(self, options={}):
        timeCondition.__init__(self, options, initial_state=True)
    def get_state(self):
        return not timeCondition.get_state(self)
        
class locationCondition(aiCondition):
    class options(conditionOptions):
        latitude = 0
        longitude = 0
        depth = 0
        use_depth = False
        error = 0.5 #meter
    def __init__(self, options, tm_info = None):
        self.tm_info = tm_info
        aiCondition.__init__(self, options)
    def get_debug_values(self):
        return {'current latitude':self.tm_info['latitude'],
                'current longitude':self.tm_info['longitude'],
                'current depth':self.tm_info['depth']}
    def get_state(self):
        return abs(self.tm_info['latitude']-self.options.latitude)<self.options.error and \
               abs(self.tm_info['longitude']-self.options.longitude)<self.options.error and \
               (abs(self.tm_info['depth']-self.options.depth)<self.options.error or not self.options.use_depth)  # pylint: disable=E1101
        
class detectorCondition(aiCondition):
    _abstract = True
    """
    This condition represents a detector
    All the options here are duplicated in the detector
    """
    def __new__(cls, *args, **kwargs):
        #need to get default options
        module = __import__('detector_library.'+cls.detector_name, fromlist=['detectorOptions'])
        opts = module.detectorOptions.get_default_options()
        cls.pipelines = ["ai"]
        cls.options = type('options', (conditionOptions, ), opts)
        return super(detectorCondition, cls).__new__(cls, *args, **kwargs)
    def __init__(self, options={}):
        aiCondition.__init__(self, options)
        self.state = False
        self.detector = None
        self._suppress_reporting = True
    def set_options(self, options):
        aiCondition.set_options(self, options)
        self.task_manager.set_detector_options(self.id, self.options.get_options())
    def on_state_set(self, state):
        self.state = state
    def register(self, task_manager):
        aiCondition.register(self, task_manager)
        #We need to tell the task manager to setup the detector, and redirect messages to this condition
        task_manager.add_detector(self.detector_name, self)
        task_manager.set_detector_options(self.id, self.options.get_options())
        self.task_manager = task_manager
    def deregister(self, task_manager):
        task_manager.remove_detector(self.id)
        aiCondition.deregister(self, task_manager)
    def get_state(self):
        return self.state

def __generateDetectorConditions():
    module_obj = sys.modules[__name__]
    #basically we want to create a whole load of new classes based on this one and some data from the detector library
    for detector_name in detector_library.__all__:
        attrs = {
            '_abstract': False,
            'detector_name': detector_name
        }
        name = detector_name+'Condition'
        setattr(module_obj, name, type(name, (detectorCondition, ), attrs))

__generateDetectorConditions()

conditions = subclassDict(aiCondition)
