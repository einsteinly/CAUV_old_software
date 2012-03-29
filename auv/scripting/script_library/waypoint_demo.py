from AI_classes import aiScript, aiScriptOptions

from cauv.debug import info, warning, debug, error

from utils.vectormath import vec

from math import atan, degrees
import time

class scriptOptions(aiScriptOptions):
    error = 0.5
    speed = 50
    checking_interval = 1.0
    class Meta:
        dynamic = ['error',]
    
class script(aiScript):
    debug_values = ['last_set.x', 'last_set.y', 'heading_to.x', 'heading_to.y', 'auv.position.x', 'auv.position.y']
    def __init__(self, *args, **kwargs):
        aiScript.__init__(self, *args, **kwargs)
        self.waypoints = []
        self.last_set = None
        self.heading_to = None
    def run(self):
        self.waypoints = (vec(0,0), vec(4.5,4.5), vec(-4.5,-6))
        #while True:
            #ans = raw_input('Mark waypoint (y/n)')
            #if ans == 'y':
                #self.waypoints.append(self.auv.position)
                #self.last_set = self.auv.position
            #elif ans == 'n':
                #break
            #else:
                #debug("Didn't enter valid answer")
        debug("Waypoints were set at "+str(self.waypoints))
        if self.auv.position == None:
            #no position data, wait for broadcast
            time.sleep(1.0)
            if self.auv.position == None:
                #if still none, then give up (since clearly not getting location
                return 'FAILURE'
        for waypoint in self.waypoints:
            self.heading_to = waypoint
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
                try:
                    bearing = degrees(atan(vector_to.x/vector_to.y))%180
                    #atan might give a result in -pi to pi
                    if vector_to.x<0:
                        bearing+180
                except ZeroDivisionError:
                    bearing = 90 if x>0 else 270
                bearing = bearing%360
                debug('Heading on a %f degree bearing, direction vector '+str(vector_to))
                self.auv.bearing(bearing)
                #go forward
                self.auv.prop(self.options.speed)
                time.sleep(self.options.checking_interval)
                vector_to = waypoint-self.auv.position
            self.auv.prop(0)
            debug("Arrived at waypoint")
        return "SUCCESS"