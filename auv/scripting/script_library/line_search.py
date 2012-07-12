from AI_classes import aiScript, aiScriptOptions, aiScriptState
from cauv.debug import debug, warning, error, info

import time

class scriptOptions(aiScriptOptions):
    bearing = 80
    useBearing = True
    depth = 2
    useDepth = True
    first_power = 100 #motor power
    first_time = 25
    power = 60
    time = 10
    repeat = 10 #repeat infinity = -1
    
class scriptState(aiScriptState):
    already_run = False

class script(aiScript):
    debug_values = ['runsRemaining', 'current_heading']
    def __init__(self, *args, **kwargs):
        aiScript.__init__(self, *args, **kwargs)
        self.runsRemaining = self.options.repeat
        self.current_heading = self.options.bearing
    def run(self):
        if self.persist.already_run:
            return
        self.persist.already_run = True
        if self.options.useDepth:
            self.auv.depthAndWait(self.options.depth)
        #first run
        if self.options.useBearing:
            self.auv.bearingAndWait(self.current_heading)
        self.auv.prop(self.options.first_power)
        time.sleep(self.options.first_time)
        self.auv.prop(0)
        self.current_heading = (self.current_heading +180)%360
        #repeat infinity if <=-1
        while self.runsRemaining != 0:
            if self.options.useBearing:
                self.auv.bearingAndWait(self.current_heading)
            self.auv.prop(self.options.power)
            time.sleep(self.options.time)
            self.auv.prop(0)
            self.runsRemaining -= 1
            self.current_heading = (self.current_heading +180)%360
        return 'FAILURE'