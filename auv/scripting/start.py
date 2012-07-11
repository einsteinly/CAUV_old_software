#from AI_classes import aiScript, aiScriptOptions, aiScriptState
#from cauv.debug import debug, warning, error, info
#from utils.boundedtypes import MotorValue

from cauv.control import AUV
from cauv.node import Node
import time

class scriptOptions():
    useDepth = True
    depth = 1.0
    to_gate_time = 24
    forward_time = 5
    back_time = 7
    forward_speed = 127
    bearing = 260
        
class scriptState():
    already_run = False

class script():
    debug_values = ['persist.already_run']
    def __init__(self):
        self.node = Node('py-start')
        self.auv = AUV(self.node)
        self.persist = scriptState
        self.options = scriptOptions
        
    def run(self):
        self.auv.bearingAndWait(self.options.bearing+90)
        if self.options.useDepth:
            self.auv.depthAndWait(self.options.depth)
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.to_gate_time)
        self.auv.prop(0)
        self.auv.bearingAndWait(self.options.bearing)
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.forward_time)
        self.auv.prop(0)
        self.auv.bearingAndWait((self.options.bearing+180)%360)
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.back_time)
        self.auv.prop(0)
        return 'SUCCESS'

s = script()
try:
    s.run()
finally:
    s.node.stop()
