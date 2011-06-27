#!/usr/bin/env python
# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node

import time
import traceback


def dfu(): 
    node = cauv.node.Node('py-dfu')
    auv = control.AUV(node)
    
    print 'setting calibration...'
    # set-up calibration factors
    node.send(msg.DepthCalibrationMessage(
        -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    ), "control")

    auv.bearingParams(1, 0, -80, 1)
    auv.depthParams(40, 0, 0, 1)
    #auv.pitchParams(1, 0, 0, 1)

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
    bearing = 325

    try:
        print 'setting bearing:'
        auv.bearing(bearing)
        time.sleep(2)

        print 'diving...'
        auv.depth(2)
        time.sleep(5)
        print 'forwards...'
        auv.prop(127)
        time.sleep(30)
        print 'stop...'
        auv.prop(0)
        time.sleep(4)
        print 'turning...'
        auv.bearing((bearing + 180) % 360)
        time.sleep(10)
        print 'forwards...'
        auv.prop(127)
        time.sleep(40)
        print 'surface...'
        auv.depth(0)
        auv.prop(0)
        time.sleep(20)
    except Exception:
        traceback.print_exc()
        auv.depth(0)
        auv.stop()
    print 'Complete'

if __name__ == '__main__':
    dfu()
