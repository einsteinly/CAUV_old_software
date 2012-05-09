#!/usr/bin/env python2.7

# Standard Library Modules
import datetime
import threading
import time

# Third Party Modules
# unfortunately this requires numpy...
from Quaternion import Quat # BSD licensed: http://pypi.python.org/pypi/Quaternion/
import numpy as np # BSD-type license

# CAUV Modules
from utils import coordinates
import cauv
import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info
from utils.hacks import tdToFloatSeconds


# If there exists a python rigid body dynamics simulator that actually works,
# please use that instead of dealing directly with quaternions!

# Note about our use of Quat: the library is geared up for astronomical angles
# (right ascension, declination, roll), we map this directly to yaw, pitch,
# roll when using the three-element constructor or anything else dealing with
# euler angles

def quatToYPR(q):
    return messaging.floatYPR(globalYaw(q), q.equatorial[2], q.equatorial[1])

def rotate(v, q):
    # no idea why this isn't working...
    #vq = Quat((v[0],v[1],v[2],0))
    #rq = vq * q.inv()
    #return (q * rq).q[:3]
    # just apply the matrix instead:
    return np.dot(q.transform, v)

def globalYaw(q):
    global_yaw = float(-q.inv().equatorial[0])
    while global_yaw < 0:
        global_yaw += 360
    return global_yaw

class MotorStates(object):
    def __init__(self):
        self.Prop = 0
        self.HBow = 0
        self.VBow = 0
        self.HStern = 0
        self.VStern = 0
    def update(self, motor_state_message):
        setattr(self,str(motor_state_message.motorId), motor_state_message.speed)
    def __repr__(self):
        return 'p=%4s hb=%4s hs=%4s vb=%4s vs=%4s' % (self.Prop, self.HBow, self.HStern, self.VBow, self.VStern)

class Model(messaging.MessageObserver):
    def __init__(self, node):
        messaging.MessageObserver.__init__(self)
        self.node = node
        self.update_frequency = 10.0
        self.datum = coordinates.Simulation_Datum
        
        # displacement in x (East), y (North), z (Altitude)
        self.displacement = np.array((0.0,0.0,-5))
        self.velocity = np.array((0.0,0.0,0.0))
        
        # Note that Quat uses degrees, NOT radians, (which is what we want)
        # the yaw component is the LOCAL YAW (about the LOCAL z axis)
        self.orientation = Quat((0.0,0.0,0.0)) # init from yaw,roll,pitch=0
        self.angular_velocity = np.array((0.0,0.0,0.0)) # dYaw/dt, dRoll/dt, dPitch/dt

        self.motor_states = MotorStates()

        self.update_lock = threading.Lock()

        self.thread = threading.Thread(target = self.runLoop)
        self.thread.daemon = False
        self.keep_going = True

        node.subMessage(messaging.MotorStateMessage())
        node.addObserver(self)

    def start(self):
        self.thread.start()
    def stop(self):
        self.keep_going = False
        self.thread.join()
    def runLoop(self):
        tprev = datetime.datetime.now()
        tdelta = datetime.timedelta(seconds=1.0/self.update_frequency)
        while self.keep_going:
            tnow = datetime.datetime.now()
            self.update_lock.acquire()
            self.processUpdate(self.motor_states)
            self.update_lock.release()
            tprev = tnow
            tnow = datetime.datetime.now()
            if tnow < tprev + tdelta:
                time.sleep(tdToFloatSeconds(tprev + tdelta - tnow))

    def onMotorStateMessage(self, m):
        debug(str(m))
        if not self.keep_going:
            return
        self.update_lock.acquire()
        self.motor_states.update(m)
        self.processUpdate(self.motor_states)
        self.update_lock.release()

    def processUpdate(self, motor_states):
        # Derived classes should implement this, and send (when appropriate):
        #   PressureMessage
        #   StateMessage
        #   and optionally: BatteryStatusMessage
        # This base class ensures that this function will be called at least 10
        # times per second, with cached motor states if nothing has changed.
        #
        # The implementation should update self.displacement, self.orientation
        # and self.velocity as appropriate.
        raise NotImplementedError('derived classes must implement this')

    def position(self):
        ned = coordinates.NorthEastDepthCoord(
            self.displacement[1], self.displacement[0], -self.displacement[2]
        )
        debug('%s' % ned, 3)
        r = self.datum + ned
        return float(r.latitude),\
               float(r.longitude),\
               float(r.altitude),\
               quatToYPR(self.orientation),\
               messaging.floatXYZ(*(float(x) for x in self.velocity))


