import cPickle
import traceback
import collections
import utils.event as event
import inspect

import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

FuncCall = collections.namedtuple("FuncCall", ['process', 'calling_process', 'function', 'args', 'kargs'])

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
    def __init__(self, process_name, manager_id=None):
        #Note that if no manager_id is oassed, then this will die quickly
        print 'initialising ' + str(self.__class__)
        #Need to be careful with use of super(), since there's now multiple
        #inheritance. It can act unexpectedly (for instance, switching the order
        #of inheritance of aiProcess will break things currently)
        super(aiProcess, self).__init__()
        #set node name
        #id = process_name[:6] if len(process_name)>6 else process_name
        self.node = cauv.node.Node("ai_"+process_name)
        self.node.subMessage(messaging.AIMessage())
        self.node.subMessage(messaging.AIlogMessage())
        self.node.subMessage(messaging.AIKeepAliveMessage())
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
        #print(self.external_functions)
        
    #this needs to be called after initialisation, as it allows other processes to message this one
    #the 'most child' process should call this at the end of its init method
    def _register(self):
        self.node.addObserver(self)

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