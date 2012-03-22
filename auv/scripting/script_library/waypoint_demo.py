from AI_classes import aiScript, aiScriptOptions

from cauv.debug import info, warning, debug, error

from math import atan, degrees

class scriptOptions(aiScriptOptions):
    error = 0.5
    class Meta:
        dynamic = ['error',]
    
class script(aiScript):
    def __init__(self, *args, **kwargs):
        aiScript.__init__(self, *args, **kwargs)
        self.waypoints = []
    def run(self):
        while True:
            ans = raw_input('Mark waypoint (y/n)')
            if ans == 'y':
                self.waypoints.append(self.auv.position)
            elif ans == 'n':
                break
            else:
                debug("Didn't enter valid answer")
        print self.waypoints
        for waypoint in self.waypoints:
            vector_to = waypoint-self.auv.position
            #             |N/y  /
            #             |    /
            #             |   /
            #             |  /
            #             | /
            #_____________|/______________x
            #             |
            #             |
            #             |
            #             |
            #             |
            #             |
            while abs(vector_to)>self.options.error:
                #rotate to vector
                bearing = degrees(atan(vector_to.x/vector_to.y))
                if vector_to.y<0:
                    bearing+180
                bearing = bearing%360
                self.auv.bearing(bearing)
                #go forward
                self.auv.prop(50)
            self.auv.prop(0)
            debug("Arrived at waypoint")
        return "SUCCESS"