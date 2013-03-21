import AI
from cauv.debug import debug, warning, error, info

import time
import traceback

#TODO ?make the AUV turn slower so that detectors get a chance to fire
#do a preliminary 360 degree sweep
class Spiral(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.loops = 10 #number of times to go round
            self.power = 60 #motor power
            self.unit = 7
            self.depth = 2
            self.stop_time = 1.5
            
    def run(self):
        self.log('Attempting spiral search...')
        # Starting search at north direction
        #debug('setting bearing %d...' % bearing)
        #self.auv.bearingAndWait(bearing)
        time.sleep(0.3)
        bearing = self.auv.getBearing()
        if bearing:
            self.auv.bearingAndWait(bearing)
        else:
            self.auv.bearingAndWait(0)
        if self.options.depth:
            self.log('Setting depth to %d' %(self.options.depth,))
            debug('diving...')
            self.auv.depthAndWait(self.options.depth, 5)

        debug('spiral...')
        # Individual half squares
        for i in range(1, 2*self.options.loops):
            debug('Performing %dth half circle' % i)
            # Individual half squares
            for j in range(2):
                # The time for which the AUV goes forward depends on the radius of the revolution
                debug('Moving forward for %d seconds' %(self.options.unit*i))
                self.auv.prop(self.options.power)
                time.sleep(self.options.unit*i)

                # Stop motor & wait for stop
                debug('stopping')
                self.auv.prop(0)
                time.sleep(self.options.stop_time)
                if bearing is None:
                    warning('no bearing available!')
                    bearing = 0

                debug('setting bearing %d' % bearing)
                bearing += 90
                if bearing>=360:
                    bearing-=360
                self.auv.bearingAndWait(bearing)
            self.log('Completed %d loops.' %(i*0.5,))

Script = Spiral

if __name__ == "__main__":
    Spiral.entry()