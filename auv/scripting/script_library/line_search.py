from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, warning, error, info

import time

class scriptOptions(aiScriptOptions):
    bearing = 80
    useBearing = True
    depth = 2
    useDepth = True
    power = 60 #motor power
    time = 10
    repeat = 2 #repeat infinity = -1

class script(aiScript):
    debug_values = ['runsRemaining', 'current_heading']
    def __init__(self, *args, **kwargs):
        aiScript.__init__(self, *args, **kwargs)
        self.runsRemaining = self.options.repeat
        self.current_heading = self.options.bearing
    def run(self):
        if self.options.useDepth:
            self.auv.depthAndWait(self.options.depth)
        #repeat infinity if <=-1
        while self.runsRemaining != 0:
            if self.options.useBearing:
                self.auv.bearingAndWait(self.current_heading)
            self.auv.prop(self.options.power)
            time.sleep(self.options.time)
            self.auv.prop(0)
            self.runsRemaining -= 1
            self.current_heading = (self.current_heading +180)%360
        return 'SUCCESS'