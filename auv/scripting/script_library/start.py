import AI
from cauv.debug import debug, warning, error, info
from utils.boundedtypes import MotorValue

import time

class Start(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.useDepth = True
            self.depth = 1.0
            self.to_gate_time = 24
            self.forward_time = 5
            self.back_time = 7
            self.forward_speed = AI.OptionWithMeta(100, opt_type=MotorValue)
            self.bearing = 260
            
    class DefaultState(AI.Script.DefaultState):
        def __init__(self):
            self.already_run = False
            
    def run(self):
        if self.persist.already_run:
            return 'SUCCESS'
        self.persist.already_run = True
        self.auv.bearingAndWait(self.options.bearing+90)
        if self.options.useDepth:
            self.log('Diving to %d to start mission' %(self.options.depth))
            self.auv.depthAndWait(self.options.depth)
        self.log('Heading forwards towards validation gate')
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.to_gate_time)
        self.log('Turning to face gate')
        self.auv.prop(0)
        self.auv.bearingAndWait(self.options.bearing)
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

Script = Start

if __name__ == "__main__":
    Start.entry()