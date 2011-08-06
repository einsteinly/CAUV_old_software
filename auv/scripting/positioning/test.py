from cauv.debug import debug, warning, error, info

from AI_location import aiLocationProvider

import time

class locationProvider(aiLocationProvider):
    
    def __init__(self, node, args):
        aiLocationProvider.__init__(self, node)
        # do something with options if you want
        error("This is just a test positioner")

    def fixPosition(self):
        # the main function for generating a position fix
        for i in range(10):
            time.sleep(1)
            if self.stopped.is_set():
                error("stop set, returning early")
                return
        info("Position fix finished")
        
    def getPosition(self):
        # return the current (or latest) position
        return (1, 2)    
