from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, warning, error, info

import time

class scriptOptions(aiScriptOptions):
    depth = 1.5
    forward_time = 60
    forward_speed = 100
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
        self.auv.depthAndWait(self.options.depth)
        self.log('Heading forwards through validation gate')
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.forward_time)
        self.auv.prop(0)
        self.notify_exit('SUCCESS')
