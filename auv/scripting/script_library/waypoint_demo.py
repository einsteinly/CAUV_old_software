from AI_classes import aiScript, aiScriptOptions

from cauv.debug import info, warning, debug, error

from utils.vectormath import vec

from math import atan, degrees
import time

class scriptOptions(aiScriptOptions):
    error = 0.5
    class Meta:
        dynamic = ['error',]
    
class script(aiScript):
    def __init__(self, *args, **kwargs):
        aiScript.__init__(self, *args, **kwargs)
        self.waypoints = []
    def run(self):
        #self.waypoints = (vec(0,0), vec(4.5,4.5), vec(-4.5,-6))
        while True:
            ans = raw_input('Mark waypoint (y/n)')
            if ans == 'y':
                self.waypoints.append(self.auv.position)
            elif ans == 'n':
                break
            else:
                debug("Didn't enter valid answer")
        print self.waypoints
        if self.auv.position == None:
            #no position data, wait for broadcast
            time.sleep(1.0)
            if self.auv.position == None:
                #if still none, then give up (since clearly not getting location
                return 'FAILURE'
        for waypoint in self.waypoints:
            vector_to = waypoint-self.auv.position
            #             |N/y  /
            #             |    /
            #             |___/
            #             |b /
            #             | /
            #_____________|/_____________x
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
                debug('Heading on a %f degree bearing, direction vector '+str(vector_to))
                self.auv.bearing(bearing)
                #go forward
                self.auv.prop(50)
                time.sleep(1.0)
                vector_to = waypoint-self.auv.position
            self.auv.prop(0)
            debug("Arrived at waypoint")
        return "SUCCESS"