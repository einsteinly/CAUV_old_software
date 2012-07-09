#!/usr/bin/env python2.7

# Standard Library Modules
import datetime
import threading
import time
import math

# Third Party Modules
import numpy as np # BSD-type license

# CAUV Modules
from utils import coordinates
from utils.quaternion import Quaternion
import cauv
import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info
from utils.hacks import tdToFloatSeconds

# If there exists a python rigid body dynamics simulator that actually works,
# please use that instead of dealing directly with quaternions!

'''
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
'''

def mkVec(x, y, z):
    return np.array((x, y, z))

def angleMean(angle_sequence):
    sins = map(math.sin, angle_sequence)
    coss = map(math.cos, angle_sequence)
    n = len(angle_sequence)
    r = math.atan2(sum(sins)/n,sum(coss)/n)
    if r < 0:
        r += 2*math.pi
    return r

def yawFromxy(x, y):
    yaw_according_to_x = math.atan2(x[1],x[0])
    yaw_according_to_y = math.atan2(-y[0],y[1])
    return angleMean((yaw_according_to_x, yaw_according_to_y))

def pitchFromxzZ(x, z, Z):
    x_cross_Z = np.cross(x,Z)
    len_x_cross_Z = sum(x_cross_Z**2)**0.5
    if len_x_cross_Z < 1e-6:
        warning('gimbal lock!')
        return 0
    return math.atan2(np.dot(z,x_cross_Z/len_x_cross_Z),np.dot(z,Z))

def rollFromyzZ(y, z, Z):
    y_cross_Z = np.cross(y,Z)
    len_y_cross_Z = sum(y_cross_Z**2)**0.5
    if len_y_cross_Z < 1e-6:
        warning('gimbal lock!')
        return 0
    return math.atan2(np.dot(z,y_cross_Z/len_y_cross_Z),np.dot(z,Z))

def calculateYPRRadians(q):
    '''Return (Yaw, Pitch, Roll) in radians. This is substantially faster (but equivalent) to using the individual methods.'''
    X = np.array((1,0,0))
    Y = np.array((0,1,0))
    Z = np.array((0,0,1))
    x = q.rotate(X, mkVec)
    y = q.rotate(Y, mkVec)
    z = q.rotate(Z, mkVec)
    yaw = yawFromxy(x, y)
    pitch = pitchFromxzZ(x, z, Z)
    roll = rollFromyzZ(y, z, Z)
    return (yaw, pitch, roll)

def calculateYaw(q):
    '''Angle in radians about global Z axis (down->up)'''
    X = np.array((1,0,0))
    Y = np.array((0,1,0))
    x = q.rotate(X, mkVec)
    y = q.rotate(Y, mkVec)
    return yawFromxy(x, y)

def calculateRoll(q):
    '''Angle from horizontal in radians about local y axis (back->front)'''
    Y = np.array((0,1,0))
    Z = np.array((0,0,1))
    y = q.rotate(Y, mkVec)
    z = q.rotate(Z, mkVec)
    return rollFromyzZ(y,z,Z)

def calculatePitch(q):
    '''Angle from horizontal in radians about local x axis (left->right): positive is up!'''
    X = np.array((1,0,0))
    Z = np.array((0,0,1))
    x = q.rotate(X, mkVec)
    z = q.rotate(Z, mkVec)
    return pitchFromxzZ(x,z,Z)

def orientationToYPR(q):
    '''Calculate the yaw, pitch and roll implied by a quaternion, and convert them to degrees'''
    yaw, pitch, roll = calculateYPRRadians(q)
    # NB: converting to our silly on-the-wire angle format here
    return messaging.floatYPR(360-math.degrees(yaw), math.degrees(pitch), math.degrees(roll))

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
    def __init__(self, node, profile=False):
        messaging.MessageObserver.__init__(self)
        self.node = node
        self.profile = profile
        self.update_frequency = 20.0
        self.datum = coordinates.Simulation_Datum
        
        # displacement in x (East), y (North), z (Altitude)
        self.displacement = np.array((0.0,0.0,-5))
        self.velocity = np.array((0.0,0.0,0.0))
        
        # Rotation applied from East,North,Altitude coordinate system to the vehicle
        # euler angles = (pitch, roll, -yaw)
        self.orientation = Quaternion.fromEuler(
            (math.radians(0), math.radians(0), math.radians(0))
        )
        #print math.degrees(calculatePitch(self.orientation))
        #assert(44 < math.degrees(calculatePitch(self.orientation)) < 46)
        #print math.degrees(calculateYaw(self.orientation))
        #assert(89 < math.degrees(calculateYaw(self.orientation)) < 91)

        # In the vehicle-local coordinate system:
        self.angular_velocity = np.array((0.0,0.0,0.0)) # about x, about y, about z

        self.motor_states = MotorStates()

        self.update_lock = threading.Lock()

        self.thread = threading.Thread(target = self.runLoopWrapper)
        self.thread.daemon = False
        self.keep_going = True

        node.subMessage(messaging.MotorStateMessage())
        node.addObserver(self)

    def start(self):
        self.thread.start()
    def stop(self):
        self.keep_going = False
        self.thread.join()
    def runLoopWrapper(self):
        if self.profile:
            import cProfile as profile
            profile.runctx('self.runLoop()', globals(), locals(), 'sim.profile')
        else:
            self.runLoop()
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
        debug(str(m), 5)
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
        #   and optionally: RedHerringBatteryStatus
        # This base class ensures that this function will be called at least 10
        # times per second, with cached motor states if nothing has changed.
        #
        # The implementation should update self.displacement, self.orientation
        # and self.velocity as appropriate.
        raise NotImplementedError('derived classes must implement this')

    def position(self):
        self.update_lock.acquire()        
        ned = coordinates.NorthEastDepthCoord(
            self.displacement[1], self.displacement[0], -self.displacement[2]
        )
        ori = self.orientation
        vel = self.velocity
        self.update_lock.release()
        debug('%s' % ned, 3)
        r = self.datum + ned
        return (float(r.latitude),
               float(r.longitude),
               float(r.altitude),
               orientationToYPR(ori),
               ori,
               messaging.floatXYZ(*(float(x) for x in vel)))


