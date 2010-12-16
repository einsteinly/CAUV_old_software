#!/usr/bin/env python
# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from cauv.debug import debug, warning, error, info

import time
import traceback


def Spiral():
    # Create a node of the CAUV messaging service
    node = cauv.node.Node('py-spiral')
    # Create a python object for the control of the AUV
    auv = control.AUV(node)

    debug('setting calibration...')
    # Set the y intercept and gradient of the pressure/depth conversion for
    # Pressure sensors
    auv.calibrateDepth(-912.2/96.2, 1.0/96.2)

    # Set kp kd ki and scale for control loops
    auv.bearingParams(1, 0, -80, 1)
    auv.depthParams(40, 0, 0, 1)
    auv.pitchParams(1, 0, 0, 1)

    auv.propMap(10, -10, 127, -127)
    auv.vbowMap(10, -10, 127, -127)
    auv.hbowMap(10, -10, 127, -127)
    auv.vsternMap(10, -10, 127, -127)
    auv.hsternMap(10, -10, 127, -127)

    time.sleep(2)
    #bearing = auv.getBearing()
    #if bearing is None:
    #    print 'no bearing information!'
    #    time.sleep(5)
    #    bearing = 90

    square=2
    bearing=0
    power=64
    unit = 3

    try:
        # Starting search at north direction
        debug('setting bearing %d...' % bearing)
        auv.bearingAndWait(bearing)

        # 2m deep
        debug('diving...')
        auv.depthAndWait(2)

        debug('spiral...')
        # Individual half squares
        for i in range(1, 2*square):
            debug('Performing %dth half circle' % i)
            # Individual half squares
            for j in range(2):
                progress = 0
                # Counter of progress in seconds
                startTime = time.time()
                debug('Moving forward and searching for %d seconds' % (3*i))
                # ...
                
                # The time for which the AUV goes forward depends on the radius of the revolution
                debug('Moving forward for %d seconds' %(3*i))
                auv.prop(power)
                time.sleep(i)

                # Stop motor & wait for stop
                debug('stopping')
                auv.prop(0)
                time.sleep(5)

                debug('setting bearing %d' % bearing)
                bearing += 90
                if bearing>=360:
                    bearing-=360
                auv.bearingAndWait(bearing)

        debug('surface...')
        auv.depthAndWait(0)


    except Exception:
        traceback.print_exc()
        auv.depth(0)
        auv.stop()
    debug('Complete')

if __name__ == '__main__':
    Spiral()




