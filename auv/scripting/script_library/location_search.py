import AI
from cauv.debug import debug, warning, error, info
import cauv.messaging as msg

from utils.coordinates import Simulation_Datum, NorthEastDepthCoord

from math import ceil, sqrt, atan, degrees
import time
    
class LocationSearch(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.locations = msg.LocationSequence([(Simulation_Datum+NorthEastDepthCoord(-5, -5, 0)).toWGS84(),
                                            (Simulation_Datum+NorthEastDepthCoord(-15, -5, 0)).toWGS84(),
                                            (Simulation_Datum+NorthEastDepthCoord(5, -10, 0)).toWGS84(),],
                                            msg.SequenceType.Path)
            self.speed = 127
            self.timeout = 120
            self.position_error = 0.5
            self.spiral_loops = 2 #number of times to go round
            self.spiral_power = 127 #motor power
            self.spiral_unit = 5
            self.spiral_stop_time = 2
            self.attempts = 5
            self.depth = 1
            self.use_depth = False
    
    class DefaultState(AI.Script.DefaultState):
        def __init__(self):
            self.checked = []
            self.partially_checked = []
            
    def run(self):
        if self.options.use_depth:
            self.auv.depth(self.options.depth)
        #choose a location from locations
        while True:
            location = None
            #choose a location to head to. preference is given to locations not yet searched
            for loc in self.options.locations.locations:
                if loc in self.persist.checked:
                    continue
                elif loc in self.persist.partially_checked:
                    location = loc
                else:
                    location = loc
                    break
            if not location:
                break
            #head to a location
            info('Heading to %d, %d' %(location.latitude, location.longitude))
            self.log('Heading to %d, %d' %(location.latitude, location.longitude))
            timeout = time.time() + self.options.timeout
            try:
                if self.auv.headToLocation(location, error = self.options.position_error, speed = self.options.speed,
                                            checking_interval = 0.5, timeout = self.options.timeout):
                    break
            except CommunicationError:
                return 'ERROR'
            #note when nearby, so don't try this location first next time
            self.persist.partially_checked.append(location)
            #spiral
            bearing = self.auv.current_bearing
            info('Spiraling to search.')
            self.log('Attempting a spiral search of location.')
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
                    time.sleep(self.options.spiral_stop_time)

                    debug('setting bearing %d' % bearing, 2)
                    bearing += 90
                    if bearing>=360:
                        bearing-=360
                    self.auv.bearingAndWait(bearing, 10)
            #record location searched
            self.persist.checked.append(location)
            self.log('Finished searching %d, %d' %(location.latitude, location.longitude))
        #give up
        info('Ran out of locations to search')
        self.log('Ran out of locations to search')
        return 'SUCCESS'

Script = LocationSearch

if __name__ == "__main__":
    LocationSearch.entry()