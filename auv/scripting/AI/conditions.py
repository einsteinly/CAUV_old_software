import sys
import threading
import detector_library
import options
import collections
import time
import weakref
import itertools

from cauv.debug import debug, warning, error, info
from utils.subclass import subclassDict

class Condition(object):
    class DefaultOptions(options.Options):
        pass
    instances = weakref.WeakValueDictionary()
    def __init__(self, ai_state, opt_dict=None, name=None):
        self.options = self.DefaultOptions()
        if opt_dict is not None:
            self.options.from_dict(opt_dict)
        self.ai_state = ai_state
        self.reset()
        _type = self.get_type()
        if name is not None:
            if name in self.instances:
                warning("Duplicate condition name {}!".format(name))
                name = None
            else:
                self.name = name
                self.instances[name] = self
        if name is None:
            for i in itertools.count(1):
                name = "{}_{}".format(_type, i)
                if name not in self.instances:
                    self.name = name
                    self.instances[name] = self
                    break

    def get_type(self):
        return self.__class__.__name__.replace("Condition","")

    def get_state(self):
        raise NotImplemented

    def reset(self):
        pass

class StateCondition(Condition):
    """
    Basic condition that just has a settable state
    """
    class DefaultOptions(options.Options):
        def __init__(self):
            self.state = False
    def get_state(self):
        return self.options.state
        
class TimeCondition(Condition):
    """
    This condition only remains true for a certain time
    """
    class DefaultOptions(options.Options):
        def __init__(self):
            self.timeout = 30
    def reset(self):
        self.start_time = time.time()
    def get_state(self):
        return self.start_time > time.time() + self.options.timeout
        
class TimeoutCondition(TimeCondition):
    def get_state(self):
        return not TimeCondition.get_state(self)
        
class LocationCondition(Condition):
    class DefaultOptions(options.Options):
        def __init__(self):
            self.latitude = 0
            self.longitude = 0
            self.depth = 0
            self.use_depth = False
            self.error = 0.5 #meter
    def get_state(self):
        return abs(self.ai_state.latitude()-self.options.latitude)<self.options.error and \
               abs(self.ai_state.longitude()-self.options.longitude)<self.options.error and \
               (abs(self.ai_state.depth()-self.options.depth)<self.options.error or not self.options.use_depth)  # pylint: disable=E1101
        
class TaskSuccessfulCondition(Condition):
    class DefaultOptions(options.Options):
        def __init__(self):
            self.task_name = ''
    def reset(self):
        self.state = False
    def get_state(self):
        #check once and store value if true as wont become false again
        if self.state:
            return True
        if self.ai_state.task_state(self.options.task_name) == "SUCCESS":
            self.state = True
            return True
        return False
        
class DetectorCondition(Condition):
    """
    This condition represents a detector
    All the options here are duplicated in the detector
    """
    def get_state(self):
        return self.ai_state.detector_fired(self.name)

def _generate_detectors():
    module_obj = sys.modules[__name__]
    #basically we want to create a whole load of new classes based on this one and some data from the detector library
    for detector_name, detector in detector_library.__dict__.iteritems():
        if not hasattr(detector, "Detector"):
            continue
        name = detector_name + 'Condition'
        detector_class = type(name, (DetectorCondition,), {})
        detector_class.detector_name = detector_name
        detector_class.DefaultOptions = detector.Detector.DefaultOptions
        detector_class.Detector = detector.Detector
        setattr(module_obj, name, detector_class)

_generate_detectors()

def get_conditions():
    conditions = subclassDict(Condition)
    del conditions['DetectorCondition']
    return {k.replace("Condition","") : v for k, v in conditions.items()}
