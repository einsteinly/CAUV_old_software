#from AI_classes import aiScript, aiScriptOptions, aiScriptState
#from cauv.debug import debug, warning, error, info
#from utils.boundedtypes import MotorValue

from cauv.control import auv
import time

class scriptOptions():
    useDepth = True
    depth = 1.0
    to_gate_time = 24
    forward_time = 5
    back_time = 7
    forward_speed = 127, MotorValue
    bearing = 260
        
class scriptState():
    already_run = False

class script(aiScript):
    debug_values = ['persist.already_run']
    def run(self):
        self.persist = scriptState
        self.options = scriptOptions
        auv.bearingAndWait(self.options.bearing+90)
        if self.options.useDepth:
            self.auv.depthAndWait(self.options.depth)
        auv.prop(self.options.forward_speed)
        time.sleep(self.options.to_gate_time)
        auv.prop(0)
        auv.bearingAndWait(self.options.bearing)
        auv.prop(self.options.forward_speed)
        time.sleep(self.options.forward_time)
        auv.prop(0)
        auv.bearingAndWait((self.options.bearing+180)%360)
        auv.prop(self.options.forward_speed)
        time.sleep(self.options.back_time)
        auv.prop(0)
        return 'SUCCESS'

script.run()
