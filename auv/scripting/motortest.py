#!/usr/bin/env python
# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from cauv.debug import debug, info, warning, error

import time
import traceback
import optparse


def setupParams(node, auv):
    info('Setting calibration parameters')
    # set-up calibration factors
    node.send(msg.DepthCalibrationMessage(
        -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    ), "control")

    auv.bearingParams(1, 0, -80, 1)
    auv.depthParams(-40, 0, 0, 1)
    auv.pitchParams(1, 0, 0, 1)

    auv.propMap(10, -10, 127, -127)
    auv.vbowMap(10, -10, 127, -127)
    auv.hbowMap(10, -10, 127, -127)
    auv.vsternMap(10, -10, 127, -127)
    auv.hsternMap(10, -10, 127, -127)


def motorTest(node, auv, power=30, delay=3, quiet = False):
    auv.stop()
    info('prop forwards:')
    auv.prop(power)
    time.sleep(delay)

    auv.stop()
    info('prop reverse:')
    auv.prop(-power)
    time.sleep(delay)

    auv.stop()
    info('hbow right:')
    auv.hbow(power)
    time.sleep(delay)
    auv.stop()
    
    auv.stop()
    info('hbow left:')
    auv.hbow(-power)
    time.sleep(delay)

    auv.stop()
    info('vbow up:')
    auv.vbow(power)
    time.sleep(delay)
    
    auv.stop()
    info('vbow down:')
    auv.vbow(-power)
    time.sleep(delay)

    auv.stop()
    info('hstern right:')
    auv.hstern(power)
    time.sleep(delay)
    
    auv.stop()
    info('hstern left:')
    auv.hstern(-power)
    time.sleep(delay)

    auv.stop()
    info('vstern up:')
    auv.vstern(power)
    time.sleep(delay)
    
    auv.stop()
    info('vstern down:')
    auv.vstern(-power)
    time.sleep(delay)

    auv.stop()
    info('Complete')

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option("-q", "--quiet", dest="quiet", default=False,
            action="store_true", help="don't print progress")
    p.add_option("-d", "--delay", dest="delay", type=float, default=3.0,
            help="pause after each motor command")
    p.add_option("-p", "--power", dest="power", help=" motor power", type=int,
            default=30)
    p.add_option("-N", "--no-params", dest="no_params", default=False,
            action="store_true",
            help="don't set motor maps and other parameters")

    p.add_option("-n", "--name", dest="name", default="py-my",
            help="CAUV Node name")
    
    opts, args = p.parse_args()
    
    node = cauv.node.Node('py-mt')
    auv = control.AUV(node)

    if not opts.no_params:
        setupParams(node, auv)

    motorTest(node, auv, opts.power, opts.delay)

