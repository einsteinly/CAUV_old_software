from AI_classes import aiScript, aiScriptOptions, aiScriptState
from cauv.debug import debug, warning, error, info
from utils.boundedtypes import MotorValue

import time

class scriptOptions(aiScriptOptions):
    depth = 1.0
    forward_time = 60
    forward_speed = 100, MotorValue
        
class scriptState(aiScriptState):
    already_run = False

class script(aiScript):
    debug_values = ['persist.already_run',]
    def run(self):
        if self.persist.already_run:
            return 'SUCCESS'
        self.persist.already_run = True
        self.log('Diving to %d to start mission' %(self.options.depth))
        self.auv.depthAndWait(self.options.depth)
        self.log('Heading forwards through validation gate')
        self.auv.prop(self.options.forward_speed)
        debug("Heading blindly in this direction until something stops me or %d seconds elapse" %(self.options.forward_time))
        time.sleep(self.options.forward_time)
        self.auv.prop(0)
        return 'SUCCESS'
