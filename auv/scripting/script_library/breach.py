import AI
from cauv.debug import debug, warning, error, info

import time

class Breach(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.depth = 4
            self.angle = 45
            self.prop = 127
            self.bearing = 260
            self.forward_time = 10
            self.vert_prop = 127
        
    class DefaultState(AI.Script.DefaultState):
        def __init__(self):
            self.already_run = False
        
    def run(self):
        if self.persist.already_run:
            return
        self.persist.already_run = True
        self.bearing(self.options.bearing)
        self.depthAndWait(self.options.depth)
        self.pitch(self.options.angle)
        self.depth(None)
        self.prop(self.options.prop)
        time.sleep(self.options.forward_time)
        self.pitch(0)
        self.prop(0)
        #attempt 2
        self.depthAndWait(self.options.depth)
        self.depth(None)
        self.pitch(None)
        self.prop(self.options.prop)
        self.vbow(self.options.vert_prop)
        self.vstern(self.options.vert_prop)
        time.sleep(self.options.forward_time)
        self.vbow(0)
        self.vstern(0)
        self.prop(0)
        self.stop()
        
Script = Breach

if __name__ == "__main__":
    Breach.entry()        