from cauv.debug import debug, warning, error, info

import options
import state
import script_library
import weakref
import itertools
import time

class TaskScript(object):
    def __init__(self, name, opt_dict = None):
        self.script_class = getattr(script_library, name).Script
        self.name = name
        self.options = self.script_class.DefaultOptions()
        if opt_dict is not None:
            self.options.from_dict(opt_dict)

class Task(object):
    instances = weakref.WeakValueDictionary()
    class DefaultOptions(options.Options):
        def __init__(self):
            self.priority = 1
            self.running_priority = 2
            self.crash_limit = 5
            self.frequency_limit = 30 # once every x seconds

    class DefaultState(state.State):
        def __init__(self):
            self.active = False
            self.paused = False
            self.crash_count = 0
            self.last_called = 0
            self.succeeded = False
            self.failed = False

    def __init__(self, conditions, script, opt_dict = None, state_dict = None, name = None):
        self.options = self.DefaultOptions()
        if opt_dict is not None:
            self.options.from_dict(opt_dict)
        self.conditions = conditions
        self.script = script
        self.state = self.DefaultState()
        if state_dict is not None:
            self.state.from_dict(state_dict)
        _type = self.script.name
        if name is not None:
            if name in self.instances:
                warning("Duplicate Task name {}!".format(name))
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

    def started(self):
        self.state.active = True
        self.state.last_called = time.time()

    def crashed(self):
        self.state.active = False
        self.state.crash_count += 1

    def stopped(self):
        self.state.active = False

    def failed(self):
        self.state.failed = True
        self.state.active = False

    def succeeded(self):
        self.state.succeeded = True
        self.state.active = False

    def priority(self):
        if self.state.active:
            return max(self.options.running_priority, self.options.priority)
        else:
            return self.options.priority

    def is_available(self):
        if self.state.failed or self.state.succeeded:
            return False
        if time.time() - self.state.last_called < self.options.frequency_limit and not self.state.active:
            return False
        if self.state.crash_count > self.options.crash_limit:
            return False
        return any((c.get_state() for c in self.conditions))
