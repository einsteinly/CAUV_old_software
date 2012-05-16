from AI_classes import aiScript, aiScriptOptions

from cauv.debug import info, warning, debug, error

from utils.vectormath import vec

from math import atan, degrees
import time

class scriptOptions(aiScriptOptions):
    error = 0.00001
    speed = 50
    checking_interval = 1.0
    class Meta:
        dynamic = ['error',]
    
class script(aiScript):
    debug_values = ['last_set_lat', 'last_set_long', 'heading_to_lat', 'heading_to_long', 'auv.latitude', 'auv.longitude']
    def __init__(self, *args, **kwargs):
        aiScript.__init__(self, *args, **kwargs)
        self.waypoints = []
        self.last_set_lat = None
        self.last_set_long = None
        self.heading_to_lat = None
        self.heading_to_long = None
    def run(self):
        self.waypoints = [(0.11779199999999999, 52.116692), (0.11757541474764843, 52.11655659246135), (0.11774114908661013, 52.116454731281465)]
        """
        while True:
            ans = raw_input('Mark waypoint (y/n)')
            if ans == 'y':
                self.waypoints.append((self.auv.longitude, self.auv.latitude))
                self.last_set = (self.auv.longitude, self.auv.latitude)
            elif ans == 'n':
                break
            else:
                debug("Didn't enter valid answer")
        """
        debug("Waypoints were set at "+str(self.waypoints))
        if self.auv.longitude == None:
            #no position data, wait for broadcast
            time.sleep(5.0)
            if self.auv.longitude == None:
                #if still none, then give up (since clearly not getting location
                return 'FAILURE'
        for waypoint in self.waypoints:
            self.heading_to_lat = waypoint[0]
            self.heading_to_long = waypoint[1]
            vector_to = (waypoint[0]-self.auv.longitude, waypoint[1]-self.auv.latitude)
            #need to check these
            #             ^N/lat+
            #             |    /
            #             |___/
            #             |b /
            #             | /
            #_____________|/_____________>E/long-
            #             |
            #             |
            #             |
            #             |
            #             |
            #             |
            while max(abs(vector_to[0]),abs(vector_to[1]))>self.options.error:
                #rotate to vector
                try:
                    bearing = degrees(atan(-vector_to[0]/vector_to[1]))%180
                    #atan gives a result in -pi/2 to pi/2
                    if vector_to[1]<0:
                        bearing+180
                except ZeroDivisionError:
                    bearing = 90 if x>0 else 270
                bearing = bearing%360
                debug('Heading on a %f degree bearing, direction vector %s' %(bearing,str(vector_to)))
                self.auv.bearing(bearing)
                #go forward
                self.auv.prop(self.options.speed)
                time.sleep(self.options.checking_interval)
                vector_to = (waypoint[0]-self.auv.longitude, waypoint[1]-self.auv.latitude)
                print waypoint, self.auv.longitude, self.auv.latitude
            self.auv.prop(0)
            debug("Arrived at waypoint")
        return "SUCCESS"
