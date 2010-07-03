#!/usr/bin/env python
# surface, whatever else happens

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node

import time
import traceback


def surface(delay):
    node = cauv.node.Node('py-surf')
    auv = control.AUV(node)

    time.sleep(delay)
    while True:
        print 'surfacing...'
        auv.stop()
        time.sleep(1)

if __name__ == '__main__':
    from optparse import OptionParser
    op = OptionParser()
    opts, args = op.parse_args()
    if len(args) != 1:
        print 'one argument (time in seconds) must be supplied'
    else:
        surface(float(args[0]))

