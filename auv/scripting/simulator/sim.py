#! /usr/bin/env python

# This program works with the 'control' program when it is run in simulation
# mode. It receives:
#   MotorStateMessage
#
# and sends:
#   PressureMessage (simulating the pressure sensors)
#   StateMessage (simulating the xsens orientation information)
#   BatteryStatusMessage (maybe..)
#
# for debug and simulation purposes, it also sends:
#   SimPositionMessage (exact position in simulated environment)
#

# Standard Library Modules
import argparse
import importlib
import time

# Third Party Modules

# CAUV Modules
import cauv
import cauv.messaging as messaging
import cauv.control as control
import cauv.node
from cauv.debug import debug, error, warning, info
import base_model

def runLoop(auv_model, node):
    auv_model.start()
    while True:
        time.sleep(0.1)
        (lt, ln, al, speed) = auv_model.position()
        node.send(SimPositionMessage(lt, ln, al, speed))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='simulate vehicle motion')
    parser.add_argument('-n', '--vehicle', dest='vehicle',
        help='name of vehicle to model', default='red-herring')

    opts  = parser.parse_args()

    vehicle_modname = opts.vehicle.lower().replace('-','') + '_model'
    vehicle_module = importlib.import_module(vehicle_modname)

    node = cauv.node.Node('py-sim')
    try:
        model = vehicle_module.Model(node)
        runLoop(model, node)
    finally:
        node.stop()
