#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.node

import aiTypes

import cPickle as pickle
import time
import threading
import math


duration = 60

def dfu():
    node = cauv.node.Node('py-dfu')
    auv = control.AUV(node)
    
    print 'setting calibration...'
    node.send(msg.DepthCalibrationMessage(
        -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    ))

    auv.bearingParams(1, 0, -80, 1)
    auv.depthPatams(-40, 0, 0, 1)
    auv.pitchParams(1, 0, 0, 1)

    auv.propMap(10, -10, 127, -127)
    auv.vbowMap(10, -10, 127, -127)
    auv.hbowMap(10, -10, 127, -127)
    auv.vsternMap(10, -10, 127, -127)
    auv.hsternMap(10, -10, 127, -127)
    
    time.sleep(2)
    bearing = auv.getBearing()
    print 'current bearing is', bearing
    
    try:
    print 'diving...' 
    auv.depth(2)
    time.sleep(5)
    
    print 'forward...'
    auv.prop(100)
    time.sleep(30)
    
    print 'stop...'
    auv.prop(0)
    time.sleep(5)

    print 'turning...'
    auv.bearing(bearing - 180)
    time.sleep(20)

    print 'forward...'
    auv.prop(100)
    time.sleep(40)

    print 'surfacing...'
    auv.depth(0)
    time.sleep(20)

if __name__ == '__main__'
    dfu()


