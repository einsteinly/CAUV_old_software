#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

#from AI_classes import aiScript, aiScriptOptions, aiScriptState
#from cauv.debug import debug, warning, error, info
#from utils.boundedtypes import MotorValue

from cauv.control import AUV
from cauv.node import Node
from cauv.debug import info, debug, warning, error
import time

class scriptOptions():
    useDepth = True
    depth = 1.0
    to_gate_time = 24
    forward_time = 5
    back_time = 7
    forward_speed = 127
    bearing = 80
        
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
        info("Turning to %d" % (self.options.bearing-90))
        self.auv.bearingAndWait(self.options.bearing-90)
        if self.options.useDepth:
            info("Diving to %d" % (self.options.depth))
            self.auv.depthAndWait(self.options.depth)
        info("Going forward")
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.to_gate_time)
        self.auv.prop(0)
        info("Turning to %d" % self.options.bearing)
        self.auv.bearingAndWait(self.options.bearing)
        info("Going forward")
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.forward_time)
        self.auv.prop(0)
        info("Turning to %d" % ((self.options.bearing+180)%360))
        self.auv.bearingAndWait((self.options.bearing+180)%360)
        info("Going forward")
        self.auv.prop(self.options.forward_speed)
        time.sleep(self.options.back_time)
        self.auv.prop(0)
        return 'SUCCESS'

s = script()
try:
    info("Started dead reckoning qualification")
    time.sleep(3)
    s.run()
finally:
    s.node.stop()
