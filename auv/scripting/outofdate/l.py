#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from cauv.debug import debug, warning, error, info

import time
import traceback


def dfu(): 
    node = cauv.node.Node('py-l')
    auv = control.AUV(node)
    bearing = 350
    try:
        time.sleep(1)
        debug("Setting shit")
        auv.autoCalibrateDepth()
        auv.depthParams(500,0.1,0,scale=-1,maxError=40)
        auv.bearingParams(3.5,0,35,scale=-1,maxError=150)
        auv.pitchParams(4,0.01,0,scale=-1,maxError=5)
        auv.pitch(0)
        auv.depth_disabled = False
        debug('set bearing')
        auv.bearing(bearing)
        time.sleep(4)
        debug('down')
        auv.depth(1)
        time.sleep(5)
        debug('forwards')
        auv.prop(127)
        time.sleep(31)
        debug('stop')
        auv.prop(0)
        time.sleep(1)
        auv.prop(-127)
        time.sleep(1)
        auv.prop(0)
        time.sleep(1)
        debug('turning')
        auv.bearing(bearing-90)
        time.sleep(5)
        debug('forwards')
        auv.prop(127)
        time.sleep(30)
        debug('stop')
        auv.prop(0)
        time.sleep(1)
        time.sleep(1)
        auv.prop(-127)
        time.sleep(1)
        auv.prop(0)
        time.sleep(1)
        debug('surfacing')
        auv.depth(0)
        time.sleep(3)
        auv.prop(0)
        time.sleep(1)
        auv.prop(-127)
        time.sleep(1)
        auv.prop(0)
        time.sleep(1)
    except Exception:
        traceback.print_exc()
        auv.depth(0)
        auv.stop() 
    finally:
        auv.depth(0)
        auv.stop()
        debug('Complete')
        auv.depth(-1)

if __name__ == '__main__':
    dfu()
