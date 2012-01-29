import cauv.messaging as messaging
from cauv.debug import info, error, debug, warning

from AI_classes import aiProcess

import cPickle
from IPython.Shell import IPShellEmbed

class aiMessageListener(messaging.MessageObserver):
    def __init__(self, node):
        messaging.MessageObserver.__init__(self)
        self.node = node
        self.node.addObserver(self)
    def onAIMessage(self, m):
        message = cPickle.loads(m.msg)
        debug('AI message: %s: %s.%s(%s, %s)' %(message[1],message[0],message[2],', '.join(map(str,message[3])),', '.join(['='.join(map(str, x)) for x in message[4].items()])))
    def die(self):
        self.node.removeObserver(self)

class aiStateListener(messaging.MessageObserver):
    def __init__(self, node):
        messaging.MessageObserver.__init__(self)
        self.node = node
        self.node.addObserver(self)
    def onTaskStateMessage(self, m):
        print m
    def onConditionStateMessage(self, m):
        print m
    def onTaskRemovedMessage(self, m):
        print m
    def onConditionRemovedMessage(self, m):
        print m
    def onConditionTypesMessage(self, m):
        print m
    def onTaskTypesMessage(self, m):
        print m
    def die(self):
        self.node.removeObserver(self)
        
def listen(ainode):
    aml = aiMessageListener(ainode.node)
    raw_input('Hit enter to stop listening.')
    aml.die()
    
def listen_state(ainode):
    if not hasattr(ainode, 'asl') or not ainode.asl:
        ainode.asl = aiStateListener(ainode.node)

def stop_listen_state(ainode):
    ainode.asl.die()
    ainode.asl = None
    
def add_task(ainode):
    task_type = raw_input('Enter task type: ')
    ainode.node.send(messaging.AddTaskMessage(task_type))
    
def remove_task(ainode):
    task_id = str(raw_input('Enter task id: '))
    ainode.node.send(messaging.RemoveTaskMessage(task_id))
    
def set_task_options(ainode):
    task_id = str(raw_input('Enter task id: '))
    task_options = input('Enter task options (as dict): ')
    ainode.node.send(messaging.SetTaskStateMessage(task_id, [], task_options, {}))
    
def set_script_options(ainode):
    task_id = str(raw_input('Enter task id: '))
    script_options = input('Enter script options (as dict): ')
    ainode.node.send(messaging.SetTaskStateMessage(task_id, [], {}, script_options))
    
def set_task_conditions(ainode):
    task_id = str(raw_input('Enter task id: '))
    conditions = input('Enter conditions (as list): ')
    ainode.node.send(messaging.SetTaskStateMessage(task_id, conditions, {}, {}))
    
def add_condition(ainode):
    condition_type = raw_input('Enter condition type: ')
    ainode.node.send(messaging.AddConditionMessage(condition_type))
    
def remove_condition(ainode):
    condition_id = str(raw_input('Enter condition id: '))
    ainode.node.send(messaging.RemoveConditionMessage(condition_id))
    
def set_condition_options(ainode):
    condition_id = str(raw_input('Enter condition id: '))
    condition_options = input('Enter condition options (as dict): ')
    ainode.node.send(messaging.SetConditionStateMessage(condition_id, condition_options))
    
def resend_data(ainode):
    ainode.node.send(messaging.RequestAIStateMessage())
    
def stop_script(ainode):
    ainode.node.send(messaging.ScriptControlMessage(messaging.ScriptCommand.Stop))
    
def shell(ainode):
    print """
    To access AI use ainode, e.g.
    -to change a condition, run ainode.ai.task_manager.notify_condition(condition_name, *args, **kwargs)
    """
    ipshell = IPShellEmbed()
    ipshell()

class option():
    def __init__(self, name, func, desc, params):
        self.name = name
        self.func = func
        self.desc = desc
        self.params = params
    def __call__(self, ainode, **kwargs):
        self.func(ainode, **kwargs)

class menu():
    def __init__(self, name, desc):
        self.name = name
        self.desc = desc
        self.params = {}
        self.options = [option('Return', lambda x:None, '', {})]
    def addFunction(self, name, func, desc, params):
        self.options.append(option(name, func, desc, params))
    def addMenu(self, menu):
        self.options.append(menu)
    def __call__(self, ainode):
        while True:
            for i, option in enumerate(self.options):
                print i,')',option.name,':',option.desc
            try:
                choice = self.options[int(raw_input('Choose an option: '))]
            except (IndexError, ValueError):
                print 'Invalid input'
                continue
            if choice.name=='Return':
                break
            params = {}
            for p in choice.params:
                try:
                    params[p] = choice.params[p](raw_input('Enter a value for parameter %s: ' %(p)))
                except:
                    print 'Invalid input'
            choice(ainode, **params)

if __name__=='__main__':
    ainode = aiProcess('remote')
    ainode._register()
    
    m = menu('Main menu', '')
    m.addFunction('Listen', listen, 'Listen to ai messages', {})
    m.addFunction('Listen to state', listen_state, 'Listen to ai state messages', {})
    m.addFunction('Stop listening to state', stop_listen_state, 'Stop listening to ai state messages', {})
    m.addFunction('Resend data', resend_data, '', {})
    
    t = menu('Task menu', '')
    t.addFunction('Add Task', add_task, '', {})
    t.addFunction('Remove Task', remove_task, '', {})
    t.addFunction('Set Task Options', set_task_options, '', {})
    t.addFunction('Set Script Options', set_script_options, '', {})
    t.addFunction('Set Task Conditions', set_task_conditions, '', {})
    
    c = menu('Condition menu', '')
    c.addFunction('Add Condition', add_condition, '', {})
    c.addFunction('Remove Condition', remove_condition, '', {})
    c.addFunction('Set Condition Options', set_condition_options, '', {})
    
    s = menu('Script menu', '')
    s.addFunction('Stop Script', stop_script, '', {})
    
    m.addMenu(t)
    m.addMenu(c)
    m.addMenu(s)
    
    try:
        m(ainode)
    finally:
        ainode.die()
