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
    bearing = getBearing()

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
