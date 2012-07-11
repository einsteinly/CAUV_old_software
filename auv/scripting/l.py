#!/usr/bin/env python2.7
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
    bearing = 165
    auv.bearing(bearing)
    try:
        print 'setting bearing:'
        time.sleep(4)
        print 'diving...'
        auv.depth(2)
        time.sleep(5)
        print 'forwards...'
        auv.prop(127)
        time.sleep(24)
        print 'stop...'
        auv.prop(0)
        time.sleep(1)
        print 'turning...'
        auv.bearing(255)
        time.sleep(5)
        print 'forwards...'
        auv.prop(127)
        time.sleep(5)
        auv.prop(0)
        auv.bearing(255+180)
        time.sleep(5)
        auv.bearing165(+180)
    except Exception:
        traceback.print_exc()
        auv.depth(0)
        auv.stop() 
    finally:
        auv.depth(0)
        auv.stop()
    print 'Complete'
    auv.depth(0)

if __name__ == '__main__':
    dfu()
