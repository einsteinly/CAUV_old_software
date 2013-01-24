#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from cauv.debug import debug, warning, error, info
from utils.subclass import subclassDict

from AI.conditions import conditions as c
from AI.base.option import aiOptions
from AI.base.script import aiScriptOptions

class taskOptions(aiOptions):
    script_name = ''
    priority = 1
    crash_count = 0
    crash_limit = 5
    frequency_limit = 30# once every x seconds
    last_called = 0
    def __init__(self, options={}):
        if not hasattr(self.__class__, 'running_priority'):
            self.running_priority = self.priority
        aiOptions.__init__(self, options)

class aiTask(object):
    class options(taskOptions):
        pass
    script_options = None
    conditions = []
    def __init__(self, options={}):
        self.registered = False
        #create instance of options
        self.options = self.__class__.options(options)
        #get script options
        self.load_script_options()
        #create instances of conditions
        self.conditions = {}
        self.persist_state = {}
        self.active = False
        self.paused = False
    def load_script_options(self):
        if self.options.script_name:
            try:
                self.script_options = __import__('script_library.'+self.options.script_name, fromlist=['scriptOptions']).scriptOptions()
            except AttributeError:
                self.script_options = aiScriptOptions()
    def register(self, task_manager):
        if self.registered:
            error('Task already setup')
            return
        self.id = task_manager.register_task(self)
        self.registered = True
    def add_default_conditions(self, task_manager):
        #any default conditions need to be added to the task manager
        condition_ids = []
        for condition_class, options in self.__class__.conditions:
            condition_ids.append(task_manager.create_condition(condition_class, options))
        task_manager.set_task_options(self.id, condition_ids=condition_ids)
    def deregister(self, task_manager):
        if not self.registered:
            error('Task not setup, so can not be deregistered')
            return
        for condition in self.conditions.itervalues():
            debug('removing task %s from condition %s' %(self.id, condition.id), 5)
            condition.task_ids.remove(self.id)
        self.registered = False
    def set_options(self, options):
        for key, value in options.items():
            setattr(self.options, key, value)
            if key == 'script_name' and value != self.options.script_name:
                #we want to reload script options
                self.load_script_options()
    def set_script_options(self, options):
        for key, value in options.items():
            setattr(self.script_options, key, value)
    def get_options(self):
        return self.options.get_options()
    def get_script_options(self):
        return self.script_options.get_options()
    def get_dynamic_options(self):
        return self.script_options.get_dynamic_options()
    def get_static_options(self):
        return self.script_options.get_static_options()
    def get_dynamic_options_as_params(self):
        return self.script_options.get_dynamic_options_as_params()
    def get_static_options_as_params(self):
        return self.script_options.get_static_options_as_params()
    def get_condition_ids(self):
        return self.conditions.keys()
    def is_available(self):
        for condition in self.conditions.values():
            if not condition.get_state():
                return False
        #don't return true if there aren't any conditions, default should be false
        if not self.conditions:
            return False
        return True
        
class start(aiTask):
    class options(taskOptions):
        script_name = 'start'
        priority = 1
        frequency_limit=1
    #need 1 condition or else won't start
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]

class test(aiTask):
    class options(taskOptions):
        script_name = 'test'
        priority = 1
        frequency_limit=1
    #need 1 condition or else won't start
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]
        
class follow_pipe(aiTask):
    class options(taskOptions):
        script_name = 'follow_pipe'
        priority = 3
    conditions = [
        (c['pipe_detectorCondition'], {}),
        ]
        
class circle_buoy(aiTask):
    class options(taskOptions):
        script_name = 'circle_buoy'
        priority = 3
    conditions = [
        #(c['buoy_detectorCondition'], {}),
        (c['stateCondition'], {'state': True}),
        ]
        
class avoid_collision(aiTask):
    class options(taskOptions):
        script_name = 'sonar_collision_avoider'
        priority = 10
        frequency_limit = 0
    conditions = [
        (c['sonar_collision_detectorCondition'], {}),
        ]
            
class track_wall(aiTask):
    class options(taskOptions):
        script_name = 'track_wall'
        priority = 3
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]

class surface(aiTask):
    class options(taskOptions):
        script_name = 'surface'
        priority = 10
    conditions = [
        (c['timeoutCondition'], {'timeout': 180, 'startTimer': True}),
        ]
        
class follow_cam(aiTask):
    class options(taskOptions):
        script_name = 'follow_cam'
        priority = 1
        frequency_limit = 10
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]
        
class location_search(aiTask):
    class options(taskOptions):
        script_name = 'location_search'
        priority = 0
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]
        
class default(aiTask):
    class options(taskOptions):
        script_name = 'spiral'
        priority = 0
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]
        
class track_asv(aiTask):
    class options(taskOptions):
        script_name = 'track_asv'
        priority = 0
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]
        
class waypoint_demo(aiTask):
    class options(taskOptions):
        script_name = 'waypoint_demo'
        priority = 2
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]
        
class line_search(aiTask):
    class options(taskOptions):
        script_name = 'line_search'
        priority = 0
        frequency_limit = 0
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]
        
class breach(aiTask):
    class options(taskOptions):
        script_name = 'breach'
        priority = 0
        frequency_limit = 0
    conditions = [
        (c['stateCondition'], {'state': True}),
        ]
        
tasks = subclassDict(aiTask)
