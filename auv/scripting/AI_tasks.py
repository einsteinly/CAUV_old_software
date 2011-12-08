from AI_conditions import conditions as c
from AI_classes import subclassDict

class taskOptions(object):
    script_name = None
    _script_options = {}
    priority = 1
    running_priority = None
    detectors_enabled_while_running = False
    crash_count = 0
    crash_limit = 5
    frequency_limit = 30# once every x seconds
    last_called = 0
    def __init__(self, options={}):
        if self.script_name:
            #we want to load script options
            script_options =  __import__('script_library.'+self.script_name, fromlist=['scriptOptions']).scriptOptions
            self._script_dynamic_options = script_options.get_dynamic_options()
            self._script_static_options = script_options.get_static_options()
        for key, attr in self.__class__.__dict__.items():
            if key[0] != '_':
                setattr(self, key, attr)
        self.__dict__.update(options)
    def __setattr__(self, attr, value):
        if attr == 'script_name' and value != self.script_name:
            if value:
                #we want to reload script options
                script_options =  __import__('script_library.'+self.script_name, fromlist=['scriptOptions']).scriptOptions
                self._script_dynamic_options = script_options.get_dynamic_options()
                self._script_static_options = get_static_options()
            else:
                #we just want to clear the old vals
                self._script_dynamic_options = script_options.get_dynamic_options()
                self._script_static_options = get_static_options()
        return object.__setattr__(self, attr, value)
    def get_options(self):
        return dict([item for item in self.__dict__.items() if item[0][0] != '_'])

class aiTask(object):
    class options(taskOptions):
        pass
    conditions = []
    def __init__(self, options={}):
        self.registered = False
        #create instance of options
        self.options = self.__class__.options(options)
        #create instances of conditions
        self.conditions = {}
        self.active = False
    def register(self, task_manager):
        if self.registered:
            error('Task already setup')
            return
        self.id = task_manager.register_task(self)
        #any default conditions need to be added to the task manager
        condition_ids = []
        for condition_class, options in self.__class__.conditions:
            condition_ids.append(task_manager.create_condition(condition_class, options))
        task_manager.set_task_options(self.id, condition_ids=condition_ids)
        self.registered = True
    def deregister(self, task_manager):
        if not self.registered:
            error('Task not setup, so can not be deregistered')
            return
        for condition in self.conditions.itervalues():
            condition.task_ids.pop(self.id)
        self.registered = False
    def set_options(self, options):
        for key, value in options.items():
            setattr(self.options, key, value)
    def set_script_options(self, options):
        for key, value in options.items():
            if key in self._script_dynamic_options:
                self.options._script_dynamic_options[key] = value
            else:
                self.options._script_static_options[key] = value
    def get_options(self):
        return self.options.get_options()
    def get_script_options(self):
        options = {}
        options.update(self.get_dynamic_options())
        options.update(self.get_static_options())
        return options
    def get_dynamic_options(self):
        return self.options._script_dynamic_options
    def get_static_options(self):
        return self.options._script_static_options
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
        detectors_enabled=True
    #need 1 condition or else won't start
    conditions = [
        (c.stateCondition, {'state': True}),
        ]
        
class follow_pipe(aiTask):
    class options(taskOptions):
        script_name = 'follow_pipe'
        priority = 3
    conditions = [
        (c.pipe_detectorCondition, {}),
        ]
        
class circle_buoy(aiTask):
    class options(taskOptions):
        script_name = 'circle_buoy'
        priority = 3
    conditions = [
        (c.buoy_detectorCondition, {}),
        ]
        
class avoid_collision(aiTask):
    class options(taskOptions):
        script_name = 'avoid_collision'
        priority = 10
    conditions = [
        (c.sonar_collision_detectorCondition, {}),
        ]
            
class track_wall(aiTask):
    class options(taskOptions):
        script_name = 'track_wall'
        priority = 3
    conditions = [
        (c.stateCondition, {'state': False}),
        ]

class surface(aiTask):
    class options(taskOptions):
        script_name = 'surface'
        priority = 10
    conditions = [
        (c.timeoutCondition, {'timeout': 180, 'startTimer': True}),
        ]
        
class follow_cam(aiTask):
    class options(taskOptions):
        script_name = 'follow_cam'
        priority = 1
    conditions = [
        (c.stateCondition, {'state': True}),
        ]
        
class default(aiTask):
    class options(taskOptions):
        script_name = 'spiral'
        priority = 0
        detectors_enabled_while_running = True
    conditions = [
        (c.stateCondition, {'state': True}),
        ]
        
tasks = subclassDict(aiTask)