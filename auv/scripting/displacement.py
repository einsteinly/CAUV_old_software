import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node
from cauv.debug import debug, error, warning, info

import time
import math

Default_Datum_Latitude = 52.116692
Default_Datum_Longitude = 0.117792

Earth_b = 6356752.3142 # polar
Earth_a = 6378137.00 # equatorial 

class XYZCoord:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def __repr__(self):
        return '(%s, %s, %s)' % (self.x, self.y, self.z)

def metresPerDegreeLongitude(lat):
    # dimensions of earth:
    tan_lat = math.tan(lat)
    beta = math.atan((Earth_b/Earth_a)*tan_lat)
    return math.pi * Earth_a * math.cos(beta) / 180

def metresPerDegreeLatitude(lng):
    return math.pi * Earth_b ** 2 / 360

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
    def __init__(self, node):
        messaging.MessageObserver.__init__(self)
        self.__node = node
        node.join("control")
        node.join("gui")
        node.join("telemetry")
        node.addObserver(self)
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
    
    # FIXME: if nothing is using this, remove it
    def getDisplacement(self):
        self.updateIntegration()
        r = math.sqrt(self.displacement.x ** 2 +
                      self.displacement.y ** 2 +
                      self.displacement.z ** 2)
        return r

    def getPositionLL(self):
        if self.displacement is None:
            return None
        self.updateIntegration()
        r = messaging.LocationMessage()
        position = self.datum + self.displacement
        r.latitude = position.latitude
        r.longitude = position.longitude
        r.altitude = position.altitude
        r.speed = messaging.floatXYZ(0, 0, 0) # TODO
        return r
    
    def updateIntegration(self):
        bearing = self.telemetry_bearing        
        if bearing is not None and self.speed is not None:
            now = time.time()
            self.displacement.x += math.sin(bearing) * self.speed
            self.displacement.y += math.cos(bearing) * self.speed
            self.time_last = now

    def onMotorStateMessage(self, m):
        debug('Motor state: %s' % m)
        if m.motorId == messaging.MotorID.Prop:
            self.updateIntegration()
            debug('Speed: %d' % m.speed)
            # hacky approximate constant -- integrate displacement3 and tune
            # constants for better version
            self.speed = m.speed / 256

    def onTelemetryMessage(self, m):
        self.telemetry_bearing = m.orientation.yaw
        self.displacement.z = -m.depth

    def onBearingAutopilotEnabledMessage(self, m):
        debug('Bearing: %d' % m.target)
        self.updateIntegration()
        self.control_bearing = m.target

if __name__ == '__main__':
    node = cauv.node.Node('py-dspl')
    auv = control.AUV(node)
    d = Displacement(node)
    while True:
        time.sleep(3)
        ll = d.getPositionLL()        
        info('%s : %s' % (d.displacement, ll))
        node.send(messaging.LocationMessage(ll))
