#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

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
    from argparse import ArgumentParser
    op = ArgumentParser()
    opts, args = op.parse_known_args()
    if len(args) != 1:
        print 'one argument (time in seconds) must be supplied'
    else:
        surface(float(args[0]))

