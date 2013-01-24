#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#


import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from cauv.debug import debug, info, warning, error


def setupParams(node, auv):
    info('Setting calibration parameters')
    auv.calibrateForFreshWater()

    auv.bearingParams(2, 0, -160, 1)
    auv.depthParams(100, 0.015, 50, 1)
    auv.pitchParams(1, 0, 0, 1)

    auv.propMap(0, 0, 127, -127)
    auv.vbowMap(0, 0, 127, -127)
    auv.hbowMap(0, 0, 127, -127)
    auv.vsternMap(0, 0, 127, -127)
    auv.hsternMap(0, 0, 127, -127)

if __name__ == '__main__':
    node = cauv.node.Node('py-setp')
    auv = control.AUV(node)
    setupParams(node, auv)


