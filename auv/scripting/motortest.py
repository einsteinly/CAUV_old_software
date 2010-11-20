#!/usr/bin/env python
# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node

import time
import traceback


def motorTest(): 
    node = cauv.node.Node('py-mt')
    auv = control.AUV(node)
    
    print 'setting calibration...'
    # set-up calibration factors
    node.send(msg.DepthCalibrationMessage(
        -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    ), "control")

    #auv.bearingParams(1, 0, -80, 1)
    #auv.depthParams(-40, 0, 0, 1)
    #auv.pitchParams(1, 0, 0, 1)

    auv.propMap(10, -10, 127, -127)
    auv.vbowMap(10, -10, 127, -127)
    auv.hbowMap(10, -10, 127, -127)
    auv.vsternMap(10, -10, 127, -127)
    auv.hsternMap(10, -10, 127, -127)
    
    delay = 5

    auv.stop()
    print 'prop forwards:'
    auv.prop(127)
    time.sleep(delay)

    auv.stop()
    print 'prop reverse:'
    auv.prop(-127)
    time.sleep(delay)

    auv.stop()
    print 'hbow right:'
    auv.hbow(127)
    time.sleep(delay)
    auv.stop()
    
    auv.stop()
    print 'hbow left:'
    auv.hbow(-127)
    time.sleep(delay)

    auv.stop()
    print 'vbow up:'
    auv.vbow(127)
    time.sleep(delay)
    
    auv.stop()
    print 'vbow down:'
    auv.vbow(-127)
    time.sleep(delay)

    auv.stop()
    print 'hstern right:'
    auv.hstern(127)
    time.sleep(delay)
    
    auv.stop()
    print 'hstern left:'
    auv.hstern(-127)
    time.sleep(delay)

    auv.stop()
    print 'vstern up:'
    auv.vstern(127)
    time.sleep(delay)
    
    auv.stop()
    print 'vstern down:'
    auv.vstern(-127)
    time.sleep(delay)

    auv.stop()
    print 'Complete'

if __name__ == '__main__':
    motorTest()



