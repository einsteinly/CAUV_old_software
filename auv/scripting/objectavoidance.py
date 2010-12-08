#!/usr/bin/env python
# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from brightnessdetect import BrightnessDetect as

import time
import traceback


def ObjectAvoidance(): 
    node = cauv.node.Node('py-objectavoid')         #Create a node of the spread messaging service
    auv = control.AUV(node)                         #Create a python object for the control of the AUV
    detect = BrightnessDetect(node, 28)                    #Turn on the colour detection script
        
    print 'setting calibration...'                  #setting the y intercept and gradient of the pressure/depth curve for front and back pressure sensor
    # set-up calibration factors
    node.send(msg.DepthCalibrationMessage(
        -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    ), "control")

    auv.bearingParams(1, 0, -80, 1)                 #Setting kp kd ki and scale of the following parameters
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

    square=2
    bearing=0
    power=64
    unit = 3
    depth =0.5 
    
    try:
        print 'setting bearing %d...' %bearing
        auv.bearingAndWait(bearing)                         #Starting search at north direction

        print 'diving...'
        auv.depthAndWait(depth)                                 #make sure it is at depth 1m
        
        print 'Moving forward'
        auv.prop(100)
        
        while true:
            if detect.detect==1:
                print 'About to hit something! reversing for 5 seconds'
                auv.prop(-127)
                time.sleep(5)
        
        print 'stoping'
        auv.prop(0)
        
        print 'surface...'    
        auv.depthAndWait(0)

        
    except Exception:
        traceback.print_exc()
        auv.depth(0)
        auv.stop()
    print 'Complete'

if __name__ == '__main__':
    ObjectAvoidance()




