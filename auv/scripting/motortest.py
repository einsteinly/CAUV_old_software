#!/usr/bin/env python
# Motor test

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from cauv.debug import debug, info, warning, error

import time
import traceback
import optparse
import random


def randomMotorTest(node, auv, max_power=127*3, duration=60.0, delay=1.0):
    auv.stop()
    debug('random motor test: %d, %d' % (max_power, duration))
    start = time.time()
    motors = ("prop", "hbow", "vbow", "hstern", "vstern")
    power_funcs = {
        "prop": auv.prop,
        "hbow": auv.hbow, "vbow": auv.vbow,
        "hstern": auv.hstern, "vstern": auv.vstern
    }
    current_speeds = {
        "prop": 0,
        "hbow": 0, "vbow": 0,
        "hstern": 0, "vstern": 0 
    }
    speeds = (-127, 127, 0, 23, 57, 103, -57, -20, -80, 79)
    def sumSpeeds():
        r = 0
        for i in current_speeds.values():
            r += abs(i)
        return r
    while time.time() < start + duration:
        limit = 100
        while limit > 0:
            limit -= 1
            motor = random.choice(motors)
            speed = random.choice(speeds)
            total_power = sumSpeeds() - abs(current_speeds[motor]) + abs(speed)
            if total_power <= max_power:
                break
        info('setting %s to %s, total power = %d' % (motor, speed, total_power))
        current_speeds[motor] = speed
        power_funcs[motor](speed)
        time.sleep(delay)
    auv.stop()

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
    p.add_option('-R', "--random", dest="random", default=False,
            action="store_true",
            help="test random motors at random speeds with at most "+\
            "RAND_POWER_LIMIT total power for RAND_DURATION seconds")
    p.add_option('-l', "--random-limit", dest="rand_power_limit", type=int)
    p.add_option('-D', "--random-duration", dest="rand_duration", type=int)


    p.add_option("-n", "--name", dest="name", default="py-my",
            help="CAUV Node name")
    
    opts, args = p.parse_args()
    
    node = cauv.node.Node('py-mt')
    auv = control.AUV(node)
    
    if opts.random:
        randomMotorTest(node, auv, opts.rand_power_limit, opts.rand_duration, opts.delay)
    else:
        motorTest(node, auv, opts.power, opts.delay)


