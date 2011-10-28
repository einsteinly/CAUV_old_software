#!/usr/bin/env python2.7
#Estimates the location of the AUV w.r.t. the reference corner of the harbour
#work in progress use at own risk

import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node
from cauv.debug import debug, error, warning, info

import time
import math
import optparse

import displacement

#coordinates of a bit of river Cam in Cambridge
#Default_Datum_Latitude = 52.116692
#Default_Datum_Longitude = 0.117792

#coordinates of start position in NURC harbour
Default_Datum_Latitude = 44.095788 
Default_Datum_Longitude = 9.865017

#principal component of Earth along polar and equatorial axis
Earth_b = 6356752.3142 # polar
Earth_a = 6378137.00 # equatorial 

class NorthEastDepthCoord:
    #coordinates in metres North, East and Down w.r.t. to the reference corner
    def __init__(self, x, y, z):
        self.north = x
        self.east = y
        self.depth = z
    def __repr__(self):
        return '(%s, %s, %s)' % (self.north, self.east, self.depth)

class XYZSpeed:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

def metresPerDegreeLongitude(lat):
    # dimensions of earth:
    tan_lat = math.tan(math.radians(lat))
    beta = math.atan((Earth_b/Earth_a)*tan_lat)
    return abs(Earth_a * math.cos(beta) * math.radians(1))

def metresPerDegreeLatitude():
    # TODO maybe improve approx; atm 1st degree
    return Earth_b * math.radians(1)

class Location(messaging.MessageObserver):
    def __init__(self, node, mode='simple'):
        messaging.MessageObserver.__init__(self)
        self.__node = node
        node.join("control")
        node.join("gui")
        node.join("telemetry")

        self.mode = mode

        # displacement estimators running in their own threads
        self.displacementEstimator = displacement.Displacement(mode = self.mode)

        self.location = NorthEastDepthCoord(0, 0, 0)
        self.timeLast = time.time()
        self.speed = XYZSpeed()
        self.depth = None
        self.control_bearing = None        
        self.telemetry_bearing = None
        node.addObserver(self)
    
    def getDisplacement(self):
        self.location.north, self.location.east = self.displacementEstimator.getDisplacement()

    def onMotorStateMessage(self, m):
        #debug('Motor state: %s' % m)
        if m.motorID in self.displacementEstimator.motors:
            speed = m.speed * self.displacementEstimator.motors[m.motorID].speedScale
            #TODO check motor directions
            if m.motorID == messaging.MotorID.Prop:
                self.speed.x = speed
            elif m.motorID == (messaging.MotorID.HBow or
                    messaging.MotorID.HStern):
                self.speed.y = speed
            elif m.motorID == (messaging.MotorID.VBow or
                    messaging.MotorID.VStern):
                self.speed.z = speed
            self.displacementEstimator.onMotorStateMessage(m)
            self.getDisplacement()

    def onTelemetryMessage(self, m):
        self.telemetry_bearing = m.orientation.yaw
        self.location.depth = m.depth
        self.displacementEstimator.onTelemetryMessage(m)
        self.getDisplacement()

    def onBearingAutopilotEnabledMessage(self, m):
        #debug('Bearing: %d' % m.target)
        self.getDisplacement()
        self.control_bearing = m.target

    def onSonarLocationMessage(self, m):
        info("Received Sonar Location Update")
        #TODO find reference wall angles w.r.t. to North and East
        self.location.north = m.position.x 
        self.location.east = m.position.y
        self.displacementEstimator.resetDisplacement()

    def getLocation(self):
        info('getting Location')
        r = messaging.LocationMessage()
        r.latitude = self.location.north
        r.longitude = self.location.east
        r.altitude = -self.location.depth
        r.speed = messaging.floatXYZ(self.speed.x, self.speed.y, self.speed.z)
        return r

if __name__ == '__main__':
    parser = optparse.OptionParser(usage='usage: %prog -m simple|exponential')
    parser.add_option("-m", "--mode", dest="mode", default="simple",
            help="integration mode: 'simple' or 'exponential' see" +
            "displacement_integrator.py for exponential integrator constants")
    opts, args = parser.parse_args()

    node = cauv.node.Node('py-dspl')
    try:
        auv = control.AUV(node)
        d = Location(node, opts.mode)
        while True:
            time.sleep(3)
            ll = d.getLocation()
            info('%s : %s' % (d.location, ll))
            node.send(messaging.LocationMessage(ll.latitude,ll.longitude,ll.altitude,messaging.floatXYZ(0, 0, 0)))
    finally:
        node.stop()
