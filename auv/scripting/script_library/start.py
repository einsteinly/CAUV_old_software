from AI_classes import aiScript, aiScriptOptions, aiScriptState
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
    class persistState(aiScriptState):
        already_run = False
    def run(self):
        if self.persist.already_run:
            return 'SUCCESS'
        self.persist.already_run = True
        self.log('Diving to %d to start mission' %(self.options.depth))
        self.auv.depthAndWait(self.options.depth)
        self.log('Heading forwards through validation gate')
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.forward_time)
        self.auv.prop(0)
        return 'SUCCESS'
