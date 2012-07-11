from AI_classes import aiScript, aiScriptOptions, aiScriptState
from cauv.debug import debug, warning, error, info
from utils.boundedtypes import MotorValue

import time

class scriptOptions(aiScriptOptions):
    useDepth = True
    depth = 1.0
    forward_time = 10
    back_time = 10
    forward_speed = 100, MotorValue
    bearing = 260
        
class scriptState(aiScriptState):
    already_run = False

class script(aiScript):
    debug_values = ['persist.already_run']
    def run(self):
        if self.persist.already_run:
            return 'SUCCESS'
        self.persist.already_run = True
        self.auv.bearingAndWait(self.options.bearing)
        if self.options.useDepth:
            self.log('Diving to %d to start mission' %(self.options.depth))
            self.auv.depthAndWait(self.options.depth)
        self.log('Heading forwards through validation gate')
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.forward_time)
        self.log('Turning around')
        self.auv.prop(0)
        self.auv.bearingAndWait((self.options.bearing+180)%360)
        self.log('Heading back through the validation gate')
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.forward_time)
        debug("Heading blindly in this direction until something stops me or %d seconds elapse" %(self.options.forward_time))
        self.auv.prop(0)
        return 'SUCCESS'
