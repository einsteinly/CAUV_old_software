from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, warning, error, info

import time
import traceback

#TODO ?make the AUV turn slower so that detectors get a chance to fire
#do a preliminary 360 degree sweep

class scriptOptions(aiScriptOptions):
    loops = 2 #number of times to go round
    power = 127 #motor power
    unit = 15
    depth = None
    stop_time = 2
    class Meta:
        dynamic = ['power', 'unit', 'stop_time']
    

class script(aiScript):
    def run(self):
        # Starting search at north direction
        #debug('setting bearing %d...' % bearing)
        #self.auv.bearingAndWait(bearing)
        bearing = self.auv.getBearing()
        if bearing:
            self.auv.bearingAndWait(bearing)
        else:
            self.auv.bearingAndWait(0)
        if self.options.depth:
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

                debug('setting bearing %d' % bearing)
                bearing += 90
                if bearing>=360:
                    bearing-=360
                self.auv.bearingAndWait(bearing)

        debug('surface...')
        self.auv.depthAndWait(0)

