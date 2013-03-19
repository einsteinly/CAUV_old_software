import AI
from cauv.debug import debug, warning, error, info

import time

class LineSearch(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.bearing = 80
            self.useBearing = True
            self.depth = 2
            self.useDepth = True
            self.first_power = 100 #motor power
            self.first_time = 25
            self.power = 60
            self.time = 10
            self.repeat = 10 #repeat infinity = -1
        
    class DefaultState(AI.Script.DefaultState):
        def __init__(self):
            self.already_run = False

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
    

Script = LineSearch

if __name__ == "__main__":
    LineSearch.entry()