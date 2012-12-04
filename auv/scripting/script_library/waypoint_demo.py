from AI.base import aiScript, aiScriptOptions

from cauv.debug import info, warning, debug, error

from utils.coordinates import LLACoord

from math import atan, degrees
import time

class scriptOptions(aiScriptOptions):
    error = 0.5
    speed = 50
    checking_interval = 1.0
    class Meta:
        dynamic = ['error',]
    
class script(aiScript):
    debug_values = ['last_set.latitude', 'last_set.longitude', 'heading_to.latitude', 'heading_to.longitude', 'auv.lla.latitude', 'auv.lla.longitude']
    def __init__(self, *args, **kwargs):
        aiScript.__init__(self, *args, **kwargs)
        self.waypoints = []
        self.last_set = None
        self.heading_to = None
    def run(self):
        #self.waypoints = [LLACoord(52.116692, 0.11779199999999999, -5), LLACoord(52.11655659246135, 0.11757541474764843, -5), LLACoord(52.116454731281465, 0.11774114908661013, -5)]
        while True:
            ans = raw_input('Mark waypoint (y/n)')
            if ans == 'y':
                self.waypoints.append(self.auv.lla)
                self.last_set = self.auv.lla
            elif ans == 'n':
                break
            else:
                debug("Didn't enter valid answer")
        debug("Waypoints were set at "+str(self.waypoints))
        if self.auv.lla == None:
            #no position data, wait for broadcast
            time.sleep(5.0)
            if self.auv.lla == None:
                #if still none, then give up (since clearly not getting location
                return 'FAILURE'
        for waypoint in self.waypoints:
            self.heading_to = waypoint
            #return northeastdepthcoord, 
            vector_to = self.auv.lla.differenceInMetresTo(waypoint)
            while max(abs(vector_to.north),abs(vector_to.east))>self.options.error:
                #rotate to vector
                #       ^N   /
                #       |   /
                #       |__/
                #       |b/
                #_______|/_________________>E
                #       |
                #       |
                #       |
                try:
                    #mod 180 to make sure in range 0, 180
                    bearing = degrees(atan(vector_to.east/vector_to.north))%180
                    #if we want to head west, need to add 180 degrees to bearing
                    if vector_to.east<0:
                        bearing+=180
                except ZeroDivisionError:
                    bearing = 90 if x>0 else 270
                debug('Heading on a %f degree bearing, direction vector %s' %(bearing,str(vector_to)))
                self.auv.prop(0)
                self.auv.bearingAndWait(bearing)
                #go forward
                self.auv.prop(self.options.speed)
                time.sleep(self.options.checking_interval)
                vector_to = self.auv.lla.differenceInMetresTo(waypoint)
                print waypoint, self.auv.lla
            self.auv.prop(0)
            debug("Arrived at waypoint")
        return "SUCCESS"
