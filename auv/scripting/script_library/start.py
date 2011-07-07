from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, warning, error, info

import time

class scriptOptions(aiScriptOptions):
    depth = 1.2
    forward_time = 30
    forward_speed = 120
    already_run = False
    class Meta:
        dynamic = ['already_run']

class script(aiScript):
    def run(self):
        if self.options.already_run:
            self.notify_exit('SUCCESS')
            return
        self.ai.task_manager.modify_task_options(self.task_name, {'already_run':True})
        self.log('Diving to %d to start mission' %(self.options.depth))
        self.auv.depthAndWait(self.options.depth, timeout=5)
        self.log('Heading forwards through validation gate')
        self.auv.prop(self.options.forward_speed)
        st = time.time()
        self.auv.depth(self.options.depth)
        while time.time()  - st < self.options.forward_time:
            time.sleep(0.5)
            self.auv.prop(self.options.forward_speed)
        self.auv.prop(0)
        self.notify_exit('SUCCESS')
        
