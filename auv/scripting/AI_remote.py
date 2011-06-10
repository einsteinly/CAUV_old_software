import cauv.messaging as messaging

from AI_classes import aiProcess

import cPickle

class aiMessageListener(messaging.BufferedMessageObserver):
    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        self.node = node
        self.node.addObserver(self)
    def onAIMessage(self, m):
        message = cPickle.loads(m.msg)
        print '%s: %s.%s(%s, %s)' %(message[1],message[0],message[2],', '.join(map(str,message[3])),', '.join(['='.join(map(str, x)) for x in message[4].items()]))
    def die(self):
        self.node.removeObserver(self)
        
def listen(ainode):
    aml = aiMessageListener(ainode.node)
    raw_input('Hit enter to stop listening.')
    aml.die()
    
def add_task(ainode, task_ref):
    ainode.ai.task_manager.add_task(task_ref)
    
def remove_task(ainode, task_ref):
    ainode.ai.task_manager.remove_task(task_ref)
    
def modify_task_option(ainode, task_ref, option_name, option_type, option_value):
    ainode.ai.task_manager.modify_task_options(task_ref, {option_name:option_type(option_value)})
    
def force_save(ainode):
    ainode.ai.task_manager.save_state()

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
            except IndexError:
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
    
    taskm = menu('Tasks', '')
    taskm.addFunction('Add task', add_task, 'Setup an (already existing) task', {'task_ref': str})
    taskm.addFunction('Remove task', remove_task, 'Unsetup an (already existing) task', {'task_ref': str})
    taskm.addFunction('Modify task option', modify_task_option, 'Change an option on an (already existing) task', {'task_ref': str, 'option_name': str, 'option_type': type, 'option_value': str})
    
    m = menu('Main menu', '')
    m.addFunction('Listen', listen, 'Listen to ai messages', {})
    m.addFunction('Force Save', force_save, 'Force the task manager to save the sucurrent state', {})
    m.addMenu(taskm)
    
    m(ainode)