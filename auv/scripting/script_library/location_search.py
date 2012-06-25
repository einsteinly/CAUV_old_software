from AI_classes import aiScript, aiScriptOptions, aiScriptState
from cauv.debug import debug, warning, error, info
import cauv.messaging as msg

from math import ceil, sqrt, atan, degrees
import time
    
class scriptOptions(aiScriptOptions):
    locations = msg.LocationSequence([],msg.SequenceType.Path)
    speed = 128
    timeout = 120
    position_error = 0.5
    spiral_loops = 2 #number of times to go round
    spiral_power = 127 #motor power
    spiral_unit = 15
    spiral_stop_time = 2
    
class scriptState(aiScriptState):
    checked = []
    partially_checked = []
    
class script(aiScript):
    def run(self):
        #choose a location from locations
        while True:
            location = None
            for loc in self.options.locations.locations:
                if loc in self.options.checked:
                    continue
                elif loc in self.options.partially_checked:
                    location = loc
                else:
                    location = loc
                    break
            if not location:
                break
            info('Heading to %d, %d' %(location.latitude, location.longitude))
            #head to a location
            self.log('Heading to %d, %d' %(location.latitude, location.longitude))
            timeout = time.time() + self.options.timeout
            self.auv.headToLocation(location, error = self.options.position_error, speed = self.options.speed,
                                    checking_interval = 0.5, timeout = self.options.timeout)
            #note when nearby, so don't try this location first next time
            self.persist.partially_checked.append(location)
            #self.ai.task_manager.modify_task_options(self.task_name, {'partially_checked': self.options.partially_checked})
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
            self.log('Finished searching %d, %d' %(location.x, location.y))
        #give up
        info('Ran out of locations to search, now surfacing')
        self.log('Ran out of locations to search, now surfacing')
        self.auv.depth(0)
