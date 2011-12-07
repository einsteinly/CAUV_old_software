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
    def die(self):
        self.node.removeObserver(self)
        
def listen(ainode):
    aml = aiMessageListener(ainode.node)
    raw_input('Hit enter to stop listening.')
    aml.die()
    
def listen_state(ainode):
    asl = aiStateListener(ainode.node)
    raw_input('Hit enter to stop listening.')
    asl.die()
    
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
    m.addFunction('Listen to State', listen_state, 'Listen to ai state messages', {})
    m.addFunction('Shell', shell, '', {})
    
    try:
        m(ainode)
    finally:
        ainode.die()