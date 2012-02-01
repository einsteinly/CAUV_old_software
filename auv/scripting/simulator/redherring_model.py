#!/usr/bin/env python2.7

# Standard Library Modules
import operator
import datetime

# Third Party Modules
import Quaternion # BSD licensed: http://pypi.python.org/pypi/Quaternion/
from Quaternion import Quat
import numpy as np # BSD-type license

# CAUV Modules
import base_model
from base_model import rotate, globalYaw
from utils.hacks import tdToFloatSeconds
import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info


# Rigid body model:
#
# Plan view:
# ==========
#
# front                                   back
#
#               origin at CoM
#                      x
#      hbow            |              hstern
#    .....|............|..............|....
#  y <----X-----red-he-o-rring--------X---->  prop
#    .....|...........................|....
#
#
#
# Side View:
# ==========
#                      z
#                      |
#      vbow            |              vstern
#     ..|..............|................|..
#  y <--X-------red-he-o-rring----------X-->  prop
#    ...|...............................|..
#
# Weight: expect to be slightly negative, taking into account buoyancy, and
# at centre of mass
#
# Moment of inertia about CG parallel to axis:
#  y-y: not modelled
#  x-x: modelled as uniform mass stick
#  z-z: modelled as uniform mass stick
#
#


# TODO: measure these, they are my educated guesses at the moment:

# Rigid body model parameters:
# Position vectors of centre of thrust, and vector of direction of thrust: (ie,
# opposite to jet of water when motor is set positive)
# All in m:
HBow_At   = np.array((0, 0.7,0)); HBow_Vec   = np.array((-1.0,   0,   0))
HStern_At = np.array((0,-0.7,0)); HStern_Vec = np.array((-1.0,   0,   0))
VBow_At   = np.array((0, 0.8,0)); VBow_Vec   = np.array((   0,   0,-1.0))
VStern_At = np.array((0,-0.8,0)); VStern_Vec = np.array((   0,   0,-1.0))
Prop_At   = np.array((0,-0.9,0)); Prop_Vec   = np.array((   0, 1.0,   0))
Mass      = 37.0      # kg
Weight    = 0.1       # N upwards (i.e., slightly buoyant)
Weight_At = np.array((0,0,0))
Length    = 1.3       # length in m, used for moment calculation only
Ixx = Izz = (Mass*(Length**2)/12.0) # kg m^2
Iyy = Ixx / 4

Max_Thrust = 22.0 # N, used in Force_Per_Unit_Thrust only
Force_Per_Unit_Thrust = Max_Thrust/127.0 # N
Max_Yaw_Moment   = 1.5*Max_Thrust*2
Max_Pitch_Moment = Max_Yaw_Moment
Max_Roll_Moment  = Max_Yaw_Moment / 20

# Hydrodynamic model:
# Drag: Newtons per metre per second, modelled from measured terminal velocity:
# (this is a simple exponential model)
Drag_F = np.array((Max_Thrust*2 / 0.3, # x (sideways)
                   Max_Thrust   / 0.4, # y (forwards)
                   Max_Thrust*2 / 0.7)) # z (up/down)
# Drag Torque: Newton metres per degree per second, measured from complete
# guesses at maximum rotation rates:
Drag_J = np.array((Max_Yaw_Moment / 45.0,   # yaw
                   Max_Roll_Moment / 45.0,  # roll
                   Max_Pitch_Moment / 5.0)) # pitch

class Model(base_model.Model):
    def __init__(self, node):
        self.tzero = None
        self.last_t = self.relativeTime()
        self.last_state_sent = self.relativeTime()
        base_model.Model.__init__(self, node)

    def relativeTime(self):
        # return relative time in floating point seconds since tzero
        if self.tzero is None:
            self.tzero = datetime.datetime.now()
        return tdToFloatSeconds(datetime.datetime.now() - self.tzero)

    def processUpdate(self, s):
        # state is stored in:
        # self.displacement = (x,y,z)   - global coordinates
        # self.velocity = (x,y,z)       - global coordinates
        # self.orientation = Quat(...)  - about vehicle centre:
        #   a_global_vec = rotate(local vec, orientation)
        #   a_local_vec = rotate(global vec, orientation.inv())
        # self.angular_velocity = (dYaw/dt, dRoll/dt, dPitch/dt)
        #
        # yaw is rotation about the z axis
        # pitch is rotation about the x axis
        # roll is rotation about the y axis
        #
        weight_vec = np.array((0,0,1))
        #debug(str(s))

        now = self.relativeTime()
        dt = now - self.last_t

        # Update velocity with forces:
        
        # Forces in vehicle-local coordinates:
        hbow_force   = HBow_Vec * s.HBow * Force_Per_Unit_Thrust
        vbow_force   = VBow_Vec * s.VBow * Force_Per_Unit_Thrust
        hstern_force = HStern_Vec * s.HStern * Force_Per_Unit_Thrust
        vstern_force = VStern_Vec * s.VStern * Force_Per_Unit_Thrust
        prop_force   = Prop_Vec * s.Prop * Force_Per_Unit_Thrust

        local_force = sum(
            (hbow_force, vbow_force, hstern_force, vstern_force, prop_force)
        )

        #debug('local force = %s' % local_force)
        
        # now in global coordinates:
        global_force = rotate(local_force, self.orientation)
        drag_force   = -Drag_F * self.velocity        
        weight_force = weight_vec * Weight
        if self.displacement[2] > 0: 
            p = min(self.displacement[2], 0.4)
            debug('auv appears to be above the surface: applying corrective weight', 3)
            weight_force = -weight_vec * 37*(p/0.4)

        force = sum((global_force, drag_force, weight_force))

        self.velocity += force * dt / Mass

        # and angular velocities with moments:
        # these moments are (about x axis, about y axis, about z axis)
        # in vehicle-local coordinates:
        hbow_moment   = np.cross(hbow_force, HBow_At)
        vbow_moment   = np.cross(vbow_force, VBow_At)
        hstern_moment = np.cross(hstern_force, HStern_At)
        vstern_moment = np.cross(vstern_force, VStern_At)
        prop_moment   = np.cross(prop_force, Prop_At) # expect this to be zero!
        weight_local_force = rotate(weight_force, self.orientation.inv())
        weight_moment = np.cross(weight_local_force, Weight_At) # this is zero anyway...
        
        local_moment = sum(
            (hbow_moment, vbow_moment,
             hstern_moment, vstern_moment,
             prop_moment,
             weight_moment)
        )
        #debug('local moment = %s' % local_moment)

        # convert moment to (yaw, roll, pitch) = (about z axis, about y axis, about x axis)

        # !!! FIXME not sure why this -ve is required here - might be because
        # yaw/roll are mixed around in the quaternion...
        local_moment = np.array((
            local_moment[2], local_moment[1], -local_moment[0]
        ))

        # drag moments:
        drag_moment = -Drag_J * self.angular_velocity

        # roll-restoring moment:
        # could model this as a pendulum with trig and stuff, but since the
        # vehicle shouldn't ever be rolling anyway...
        roll = self.orientation.equatorial[1]
        # handle wrap-around from 360-0
        while roll > 180:
            roll = roll - 360
        roll_moment = np.array((0, -self.angular_velocity[1]-2*roll, 0))

        moment = sum((local_moment, drag_moment, roll_moment))

        # apply moment to angular velocity:
        domega = moment * dt / np.array((Izz, Iyy, Ixx))
        self.angular_velocity += domega

        # Update positions with velocities:
        self.displacement += self.velocity * dt
        dorientation = Quat(self.angular_velocity * dt)
        self.orientation *= dorientation

        self.last_t = now

        # last thing: renormalise orientation:
        normalised_orientation  = Quat(Quaternion.normalize(self.orientation.q))
        if sum((normalised_orientation.equatorial -
                self.orientation.equatorial) ** 2) > 0.0001:
            debug('orientation quat denormalised: it will be renormalised')
            self.orientation = self.normalised_o
        
        # last last thing: send messages reflecting the new state: this has to
        # be rate-limited since it can cause more motor state message to be
        # sent from control - leading to a tight loop
        if self.relativeTime() - self.last_state_sent > 0.05:
            self.sendStateMessages()
            self.last_state_sent = self.relativeTime()

    def sendStateMessages(self):
        # send:
        #   PressureMessage
        #   StateMessage
        #   should maybe also send BatteryStatusMessage
        # 
        # These should match the DepthCalibration message used:
        depth_offset = -9.48232
        depth_mult = 0.010395
        depth = float(-self.displacement[2])
        pressure = int((depth - depth_offset) / depth_mult)
        # if somehow we're floating above the surface...
        if pressure < 0:
            pressure = 0
        
        # on-the-wire convention for bearing is positive-clockwise
        global_yaw = globalYaw(self.orientation)
        pitch = float(self.orientation.equatorial[2])
        roll = float(self.orientation.equatorial[1])
        orientation = messaging.floatYPR(global_yaw, pitch, roll)
        #print 'pressure=', pressure, 'orientation=', orientation
        self.node.send(messaging.PressureMessage(pressure, pressure))
        self.node.send(messaging.StateMessage(orientation))




