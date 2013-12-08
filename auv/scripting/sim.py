#! /usr/bin/env python2.7

# This program works with the 'control' program when it is run in simulation
# mode. It receives:
#   MotorStateMessage
#
# and sends:
#   PressureMessage (simulating the pressure sensors)
#   StateMessage (simulating the xsens orientation information)
#
# for debug and simulation purposes, it also sends:
#   SimPositionMessage (exact position in simulated environment)
#

# Standard Library Modules
import argparse
import importlib
import time

# Third Party Modules
import rospy

# CAUV Modules
import cauv
import cauv.control as control
from cauv.debug import debug, error, warning, info
import simulator.base_model

def fmtArr(a):
    r = ''
    for x in a:
        r += '%.4f,' % float(x)
    if len(r):
        return '[' + r[:-1] + ']'
    else:
        return '[]'

def runLoop(auv_model):
    auv_model.start()
    while not rospy.is_shutdown():
        # this is just a print-loop, the work is done in a separate thread,
        # running at a configurable tick-rate
        time.sleep(5)
        (lt, ln, al, oriYPR, ori, speed) = auv_model.position()
        #debug('lat:%.12g lon:%.12g alt:%.12g' % (lt, ln, al), 3)
        #node.send(messaging.SimPositionMessage(messaging.WGS84Coord(lt, ln, al), oriYPR, 
        #                messaging.quat(ori.q0, ori.q1, ori.q2, ori.q3), speed))

        info('displ=%s\tvel=%s\tori=%s\tomega=%s\t' %
            (fmtArr(auv_model.displacement),
             fmtArr(auv_model.velocity),
             oriYPR,
             fmtArr(auv_model.angular_velocity))
        )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='simulate vehicle motion')
    parser.add_argument('-n', '--vehicle', dest='vehicle',
        help='name of vehicle to model', default='barracuda')
    parser.add_argument('-p', '--profile', dest='profile', action='store_true')
    parser.add_argument('-x', '--currentx', type=float, dest='currentx', default=0)
    parser.add_argument('-y', '--currenty', type=float, dest='currenty', default=0)
    parser.add_argument('-z', '--currentz', type=float, dest='currentz', default=0)
	    

    opts,args  = parser.parse_known_args()

    vehicle_modname = 'simulator.' + opts.vehicle.lower().replace('-','') + '_model'
    vehicle_module = importlib.import_module(vehicle_modname)

    rospy.init_node("Sim")
    model = None
    try: 
        model = vehicle_module.Model(profile=opts.profile, currentx=opts.currentx, currenty=opts.currenty, currentz=opts.currentz)
        runLoop(model)
    finally:
        if model is not None:
            model.stop()
