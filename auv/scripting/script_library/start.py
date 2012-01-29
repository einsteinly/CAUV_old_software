from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, warning, error, info
from utils.boundedtypes import MotorValue

import time

class scriptOptions(aiScriptOptions):
    depth = 1.0
    forward_time = 60
    forward_speed = 100, MotorValue
    already_run = False, bool
    class Meta:
        dynamic = ['already_run']

class script(aiScript):
    def run(self):
        if self.options.already_run:
            return 'SUCCESS'
        self.ai.task_manager.modify_script_options(self.task_name, {'already_run':True})
        self.log('Diving to %d to start mission' %(self.options.depth))
        self.auv.depthAndWait(self.options.depth)
        self.log('Heading forwards through validation gate')
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.forward_time)
        self.auv.prop(0)
        return 'SUCCESS'
