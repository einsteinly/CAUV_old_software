from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, warning, error, info

from math import ceil, sqrt, atan, degrees
import time

class scriptOptions(aiScriptOptions):
    locations = []
    #search_area = None
    speed_p = 1000
    speed_d = 0.125
    checked = []
    partially_checked = []
    class Meta:
        dynamic = ['checked', 'partially_checked']
    
class Vector():
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __add__(self, other):
        return Vector(self.x+other.x,self.y+other.y)
    def __sub__(self, other):
        return Vector(self.x-other.x,self.y-other.y)
    def __eq__(self, other):
        if self.x==other.x and self.y==other.y:
            return True
        return False
    def __gt__(self, other):
        if self.x>other.x and self.y>other.y:
            return True
        return False
    def __lt__(self, other):
        if self.x<other.x and self.y<other.y:
            return True
        return False
    def round(self):
        return Vector(round(self.x), round(self.y))
    def magnitude(self):
        return sqrt(self.x**2 + self.y**2)
    def bearing(self):
        if self.x>0:
            return 90 - degrees(atan(self.y/self.x))
        elif self.x<0:
            return 270 - degrees(atan(self.y/self.x))
        return 0 if self.y>0 else 180
"""    
class Area():
    def __init__(self, v1, v2):
        if not v2>v1:
            raise Exception('coord2 must be > coord1')
        self.coords = (v1, v2)
        
class searchArea(Area):
    def __init__(self, v1, v2, search_resolution=1):
        Area.__init__(self, v1, v2)
        self.search_grid = [[False for y in range(int(ceil((self.v2.y-self.v1.y)/search_resolution)))] for x in range(int(ceil((self.v2.x-self.v1.x)/search_resolution)))]
    def areaSearched(area):
        low_corner_rel = int((area.coords[0] - self.coords[0]).round())
        high_corner_rel = int((area.coords[0] - self.coords[0]).round())
        if low_corner_rel.x<0:
            low_corner_rel.x = 0
        if low_corner_rel.y<0:
            low_corner_rel.y = 0
        if high_corner_rel.x>len(self.grid):
            high_corner_rel.x = len(self.grid)
        if high_corner_rel.y>len(self.grid[0]):
            high_corner_rel.y = len(self.grid[0])
        for x in range(low_corner_rel.x, high_corner.x):
            for y in range(low_corner_rel.y, high_corner.y):
                self.grid[x][y] = True
        info('%d percent of area searched.')
"""
    
class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts)
    def run(self):
        #choose a location from locations
        while True:
            location = None
            for loc in self.options.locations:
                if loc in self.options.checked:
                    continue
                elif loc in self.options.partially_checked:
                    location = loc
                else:
                    location = loc
                    break
            if not location:
                break
            info('Heading to %d, %d' %(location.x, location.y))
            #head to a location
            self.log('Heading to %d, %d' %(location.x, location.y))
            timeout = time.time() + self.options.timeout
            while True:
                self.err = err = Vector(self.auv.latitude, self.auv.longitude) - location
                if err.magnitude()<self.options.pos_e or (err.magnitude()<self.options.pos_max_e and timeout<time.time()):
                    break
                self.auv.bearing(err.bearing())
                if hasattr(self, 'err'):
                    derr = err-self.err #ignore time, since relatively const
                else:
                    derr = Vector(0, 0)
                self.err = err
                speed = self.options.speed_p*(err.magnitude()-self.options.speed_d*derr.magnitude())
                speed = -127 if int(speed) <= -127 else (int(speed) if speed<=127 else 127)
                self.auv.prop(speed)
                self.sleep(0.5)
            self.auv.prop(0)
            #note when nearby, so don't try this location first next time
            self.ai.task_manager.modify_options(self.task_name, {'partially_checked': location})
            #spiral
            bearing = self.auv.current_bearing
            info('Spiraling to search')
            self.log('Attempting a spiral search of location')
            # Individual half squares
            for i in range(1, 2*self.options.spiral_loops):
                debug('Performing %dth half of spiral' % i)
                # Individual half squares
                for j in range(2):
                    # The time for which the AUV goes forward depends on the radius of the revolution
                    debug('Moving forward for %d seconds' %(self.options.spiral_unit*i,), 2)
                    self.auv.prop(self.options.spiral_power)
                    time.sleep(self.options.spiral_unit*i)

                    # Stop motor & wait for stop
                    debug('stopping', 2)
                    self.auv.prop(0)
                    time.sleep(self.options.stop_time)

                    debug('setting bearing %d' % bearing, 2)
                    bearing += 90
                    if bearing>=360:
                        bearing-=360
                    self.auv.bearingAndWait(bearing, 10)
            #record location searched
            self.checked.add(location)
            #TODO mark area as searched
        #TODO determine areas not searched
        #give up
        self.log('Ran out of locations to search, now surfacing')
        self.auv.depth(0)