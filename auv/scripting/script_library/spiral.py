from AI_classes import aiScript
from cauv.debug import debug, warning, error, info

import time
import traceback

class script(aiScript):
    def run(self):
        square=2
        bearing=0
        power=64
        unit = 3
        # Starting search at north direction
        debug('setting bearing %d...' % bearing)
        self.auv.bearingAndWait(bearing)

        # 2m deep
        debug('diving...')
        self.auv.depthAndWait(2)

        debug('spiral...')
        # Individual half squares
        for i in range(1, 2*square):
            debug('Performing %dth half circle' % i)
            # Individual half squares
            for j in range(2):
                # The time for which the AUV goes forward depends on the radius of the revolution
                debug('Moving forward for %d seconds' %(3*i))
                self.auv.prop(power)
                time.sleep(i)

                # Stop motor & wait for stop
                debug('stopping')
                self.auv.prop(0)
                time.sleep(5)

                debug('setting bearing %d' % bearing)
                bearing += 90
                if bearing>=360:
                    bearing-=360
                self.auv.bearingAndWait(bearing)

        debug('surface...')
        self.auv.depthAndWait(0)

