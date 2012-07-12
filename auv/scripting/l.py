#!/usr/bin/env python2.7
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
        debug('set bearing')
        auv.bearing(bearing)
        time.sleep(4)
        debug('down')
        auv.depth(2)
        time.sleep(5)
        debug('forwards')
        auv.prop(127)
        time.sleep(24)
        debug('stop')
        auv.prop(0)
        time.sleep(1)
        debug('turning')
        auv.bearing(bearing-90)
        time.sleep(5)
        debug('forwards')
        auv.prop(127)
        time.sleep(8)
        debug('stop')
        auv.prop(0)
        time.sleep(1)
        debug('turning')
        auv.bearing(bearing-180)
        time.sleep(70)
        debug('surfacing')
        auv.depth(0)
        time.sleep(3)
        auv.prop(0)
    except Exception:
        traceback.print_exc()
        auv.depth(0)
        auv.stop() 
    finally:
        auv.depth(0)
        auv.stop()
    debug('Complete')
    auv.depth(0)

if __name__ == '__main__':
    dfu()
