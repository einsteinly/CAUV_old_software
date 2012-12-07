import threading
import time

from cauv.debug import debug, warning, error, info
from cauv import messaging

from math import atan, degrees

class CommunicationError(Exception):
    pass

class fakeSonarfunction():
    def __init__(self, script, attr):
        self.script = script
        self.attr = attr
    def __call__(self, *args, **kwargs):
        self.script.ai.auv_control.sonar_command(self.attr, *args, **kwargs)
    def __getattr__(self, attr):
        error('You can only call functions of sonar, one level deep')
        raise AttributeError('fakeSonarfunction has no attribute %s' %(attr,))

class fakeSonar():
    def __init__(self, script):
        self.script = script
    def __getattr__(self, func):
        return fakeSonarfunction(self.script, func)

class fakeAUVfunction():
    def __init__(self, script, attr):
        self.script = script
        self.attr = attr
    def __call__(self, *args, **kwargs):
        debug('fakeAUVfunction: __call__ args=%s kwargs=%s' % (str(args), str(kwargs)), 5)
        self.script.ai.auv_control.auv_command(self.attr, *args, **kwargs)
    def __getattr__(self, attr):
        error('You can only call functions of AUV, one level deep (except sonar)')
        raise AttributeError('fakeAUVfunction has no attribute %s' %(attr,))

class fakeAUV(messaging.MessageObserver):
    #TODO
    #needs to respond to control overriding commands
    def __init__(self, script):
        self.script = script #passing the script saves on the number of AI_process, as fakeAUV can now call other processes through the script
        self.sonar = fakeSonar(script)
        messaging.MessageObserver.__init__(self)
        self.script.node.subMessage(messaging.TelemetryMessage())
        self.script.node.addObserver(self)
        self.current_bearing = None
        self.current_depth = None
        self.current_pitch = None
        self.lla = None
        #self.altitude = None
        #self.speed = None
        self.bearingCV = threading.Condition()
        self.depthCV = threading.Condition()
        self.pitchCV = threading.Condition()
        
    def onTelemetryMessage(self, m):
        #self.bearing = m.orientation.yaw
        self.current_bearing = m.orientation.yaw
        self.current_depth = m.depth
        self.current_pitch = m.orientation.pitch
        with self.bearingCV:
            with self.depthCV:
                with self.pitchCV:
                    self.bearingCV.notifyAll()
                    self.depthCV.notifyAll()
                    self.pitchCV.notifyAll()
        
    #def onLocationMessage(self, m):
    #self.latitude = m.location.latitude
    #self.longitude = m.location.longitude
    #self.altitude = m.location.altitude
    #self.speed = m.speed
    
    def getBearing(self):
        return self.current_bearing
    
    def bearingAndWait(self, bearing, epsilon = 5, timeout = 20):
        if bearing == None:
            self.bearing(None)
            return True
        startTime = time.time()
        self.bearing(bearing)
        while time.time() - startTime < timeout:
            if self.current_bearing == None or min((bearing - self.current_bearing) % 360, (self.current_bearing - bearing) % 360) > epsilon:
                #print 'bearing waiting'
                with self.bearingCV:
                    self.bearingCV.wait(timeout - time.time() + startTime)
            else:
                return True
        return False
        
    def depthAndWait(self, depth, epsilon = 5, timeout = 20):
        if depth == None:
            self.depth(None)
            return True
        startTime = time.time()
        self.depth(depth)
        while time.time() - startTime < timeout:
            if self.current_depth == None or abs(depth - self.current_depth) > epsilon:
                with self.depthCV:
                    self.depthCV.wait(timeout - time.time() + startTime)
            else:
                return True
        return False

    def pitchAndWait(self, pitch, epsilon = 5, timeout = 20):
        if pitch == None:
            self.pitch(None)
            return True
        startTime = time.time()
        self.pitch(pitch)
        while time.time() - startTime < timeout:
            if self.current_pitch == None or min((pitch - self.current_pitch) % 360, (self.current_pitch - pitch) % 360) > epsilon:
                with self.pitchCV:
                    self.pitchCV.wait(timeout - time.time() + startTime)
            else:
                return True
        return False
        
    def headToLocation(self, target_lla, depth_enabled = False, error = 0.5, speed = 127, checking_interval = 0.5, timeout = None):
        if not self.lla:
            time.sleep(1) #tends to happen if task has only just started
        if self.lla:
            vector_to = self.lla.differenceInMetresTo(target_lla)
        else:
            raise CommunicationError('No location data received yet.')
        if depth_enabled:
            self.depth(self.current_depth+self.lla.altitude-target_lla.altitude)
        if timeout:
            end_time = time.time() + timeout
        while True:
            if max(abs(vector_to.north),abs(vector_to.east))<error:
                self.prop(0)
                return True
            elif timeout:
                if end_time < time.time():
                    self.prop(0)
                    return False
            #rotate to vector
            #       ^N   /
            #       |   /
            #       |__/
            #       |b/
            #_______|/_________________>E
            #       |
            #       |
            #       |
            try:
                #mod 180 to make sure in range 0, 180
                bearing = degrees(atan(vector_to.east/vector_to.north))%180
                #if we want to head west, need to add 180 degrees to bearing
                if vector_to.east<0:
                    bearing+=180
            except ZeroDivisionError:
                bearing = 90 if vector_to.east>0 else 270
            debug('Heading on a %f degree bearing, direction vector %s' %(bearing,str(vector_to)))
            self.prop(0)
            self.bearingAndWait(bearing)
            #go forward
            self.prop(speed)
            time.sleep(checking_interval)
            vector_to = self.lla.differenceInMetresTo(target_lla)
        
    def __getattr__(self, attr):
        debug('FakeAUV: returning dynamic override for attr=%s' % str(attr), 3)
        return fakeAUVfunction(self.script, attr)