import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node
from cauv.debug import debug, error, warning, info

import time
import math
import optparse

import displacement_integrator

Metres_Per_Second_Per_Motor_Unit = 0.4 / 127 #empirical estimate

#coordinates of a bit of river Cam in Cambridge
Default_Datum_Latitude = 52.116692
Default_Datum_Longitude = 0.117792

#principal component of Earth along polar and equatorial axis
Earth_b = 6356752.3142 # polar
Earth_a = 6378137.00 # equatorial 

radiansPerDegree = math.pi / 180

class XYZCoord:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def __repr__(self):
        return '(%s, %s, %s)' % (self.x, self.y, self.z)

def metresPerDegreeLongitude(lat):
    # dimensions of earth:
    # TODO check for negative numbers
    tan_lat = math.tan(radiansPerDegree * lat)
    beta = math.atan((Earth_b/Earth_a)*tan_lat)
    return Earth_a * math.cos(beta) * radiansPerDegree 

def metresPerDegreeLatitude(lng):
    # TODO maybe improve approx; atm 1st degree
    return Earth_b * radiansPerDegree

class LLACoord:
    def __init__(self, lat, lng, alt):
        self.latitude = lat
        self.longitude = lng
        self.altitude = alt
    def __add__(self, other):
        new_lat = self.latitude  + other.x / metresPerDegreeLatitude(self.longitude)
        new_lng = self.longitude + other.y / metresPerDegreeLongitude(self.latitude)
        new_alt = self.altitude + other.z
        return LLACoord(new_lat, new_lng, new_alt)
    def __repr__(self):
        return '(%s, %s, %s)' % (self.latitude, self.longitude, self.altitude)

class Displacement(messaging.MessageObserver):
    def __init__(self, node, mode='simple'):
        messaging.MessageObserver.__init__(self)
        self.__node = node
        node.join("control")
        node.join("gui")
        node.join("telemetry")

        self.mode = mode

        # exponential integration, runs its own thread
        self.integrator = displacement_integrator.DisplacementIntegrator()

        # simple integration
        self.displacement = XYZCoord(0, 0, 0)
        self.timeLast = time.time()
        self.speed = None
        self.depth = None
        self.control_bearing = None        
        self.telemetry_bearing = None

        self.datum = LLACoord(
            Default_Datum_Latitude,
            Default_Datum_Longitude,
            0
        )

        node.addObserver(self)
    
    def getDisplacement(self):
        self.updateIntegration()
        if self.mode == 'simple':
            r = self.displacement
        else:
            t = self.integrator.getDisplacement()
            r = XYZCoord(t[0], t[1], t[2])
        return r

    def getPositionLL(self):
        d = self.getDisplacement()
        if d is None:
            return None
        info('getPositionLL: displacement=%s' % str(d))
        self.updateIntegration()
        r = messaging.LocationMessage()
        position = self.datum + d
        r.latitude = position.latitude
        r.longitude = position.longitude
        r.altitude = position.altitude
        r.speed = messaging.floatXYZ(0, 0, 0) # TODO
        return r
    
    def updateIntegration(self):
        bearing = self.telemetry_bearing        
        if bearing is not None and self.speed is not None:
            now = time.time()
            self.displacement.x += math.sin(radiansPerDegree * bearing) * self.speed
            self.displacement.y += math.cos(radiansPerDegree * bearing) * self.speed
            self.time_last = now

    def onMotorStateMessage(self, m):
        #debug('Motor state: %s' % m)
        if m.motorId == messaging.MotorID.Prop:
            self.updateIntegration()
            debug('Speed: %d' % m.speed)
            self.speed = m.speed * Metres_Per_Second_Per_Motor_Unit
        self.integrator.onMotorStateMessage(m)

    def onTelemetryMessage(self, m):
        self.telemetry_bearing = m.orientation.yaw
        self.displacement.z = -m.depth
        self.integrator.onTelemetryMessage(m)

    def onBearingAutopilotEnabledMessage(self, m):
        #debug('Bearing: %d' % m.target)
        self.updateIntegration()
        self.control_bearing = m.target

    def onGPSLocationMessage(self, m):
        info("Received GPS Update")
        # change the fixed point
        self.datum = LLACoord(m.latitude, m.longitude, m.altitude)
        # reset the displacements to 0
        self.displacement.x = 0;
        self.displacement.y = 0;
        self.displacement.z = 0;
        self.integrator.resetDisplacement();

if __name__ == '__main__':
    parser = optparse.OptionParser(usage='usage: %prog -m simple|exponential')
    parser.add_option("-m", "--mode", dest="mode", default="exponential",
            help="integration mode: 'simple' or 'exponential' see" +
            "displacement_integrator.py for exponential integrator constants")
    opts, args = parser.parse_args()

    node = cauv.node.Node('py-dspl')
    auv = control.AUV(node)
    d = Displacement(node, opts.mode)
    while True:
        time.sleep(3)
        ll = d.getPositionLL()
        info('%s : %s' % (d.displacement, ll))
        node.send(messaging.LocationMessage(ll.latitude,ll.longitude,ll.altitude,messaging.floatXYZ(0, 0, 0)))
