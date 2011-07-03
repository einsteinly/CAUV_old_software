import cauv.messaging as messaging

from AI_classes import aiProcess

import cPickle
from IPython.Shell import IPShellEmbed

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
    
def modify_task_options(ainode, task_ref):
    print 'Please be careful, this field executes python you write in it'
    opt_dict = input('Please enter a dictionary of option_name: value pairs: ')
    ainode.ai.task_manager.modify_task_options(task_ref, opt_dict)
    
def export_task_data(ainode, file_name):
    ainode.ai.task_manager.export_task_data(file_name)
    
def force_save(ainode):
    ainode.ai.task_manager.save_state()
    
def stop_script(ainode):
    ainode.ai.task_manager.request_stop_script()
    
def enable_control(ainode):
    ainode.ai.auv_control.enable()
    
def disable_control(ainode):
    ainode.ai.auv_control.disable()
    
def add_request(ainode, pipeline_name):
    ainode.ai.pipeline_manager.request_pl('other', 'airemote', pipeline_name)
    
def drop_request(ainode, pipeline_name):
    ainode.ai.pipeline_manager.drop_pl('other', 'airemote', pipeline_name)
    
def export_pls(ainode):
    ainode.ai.pipeline_manager.export_pipelines()
    
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
    taskm.addFunction('Modify task options', modify_task_options, 'Change an options on an (already existing) task', {'task_ref': str})
    taskm.addFunction('Export task data', export_task_data, 'Save options etc to file.', {'file_name': str})
    
    scriptm = menu('Script', '')
    scriptm.addFunction('Stop script', stop_script, 'Stop the current script (if conditions true may immediately restart)', {})
    scriptm.addFunction('Enable script control', enable_control, 'Allow scripts to move the AUV', {})
    scriptm.addFunction('Disable script control', disable_control, 'Stop scripts from moving the AUV', {})
    
    imgm = menu('Image Pipeline', '')
    imgm.addFunction('Add request', add_request, '', {'pipeline_name': str})
    imgm.addFunction('Drop request', drop_request, '', {'pipeline_name': str})
    imgm.addFunction('Export Pipelines', export_pls, '', {})
    
    m = menu('Main menu', '')
    m.addFunction('Listen', listen, 'Listen to ai messages', {})
    m.addFunction('Force Save', force_save, 'Force the task manager to save the current state', {})
    m.addMenu(taskm)
    m.addMenu(scriptm)
    m.addMenu(imgm)
    m.addFunction('Shell', shell, '', {})
    
    m(ainode)