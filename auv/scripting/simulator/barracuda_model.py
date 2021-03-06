#!/usr/bin/env python2.7

# Standard Library Modules 
import operator
import datetime
import math
import random

# Third Party Modules
import numpy as np # BSD-type license
import rospy

# CAUV Modules
import base_model
from base_model import calculateRoll, mkVec
from utils.hacks import tdToFloatSeconds
from utils.quaternion import Quaternion

from cauv.debug import debug, error, warning, info
import std_msgs.msg as std_msgs
import cauv_control.msg as ctrl_msgs

# Rigid body model:
#
# Vehicle coordinate system: y is forwards, x is right, z is up
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
#  y <----X-----barracuda--------X---->  prop
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
#  y <--X-------barracuda----------X-->  prop
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
HBow_At   = np.array((0, 0.32,0)); HBow_Vec   = np.array((-1.0,   0,   0))
HStern_At = np.array((0,-0.32,0)); HStern_Vec = np.array((-1.0,   0,   0))
VBow_At   = np.array((0, 0.41,0)); VBow_Vec   = np.array((   0,   0,1.0))
VStern_At = np.array((0,-0.41,0)); VStern_Vec = np.array((   0,   0,1.0))
Prop_At   = np.array((0,-0.73,0)); Prop_Vec   = np.array((   0, 1.0,   0))
Mass      = 25.0      # kg
Displacement = 25.01   # kg
Weight_At = np.array((0,0,-0.1))
Buoyancy_At = np.array((0,0,0.05))
Length    = 1.53      # length in m, used for moment calculation only
Ixx = Izz = (Mass*(Length**2)/12.0) # kg m^2 (a uniform rod)
Iyy = Ixx / 4
Ixx_water = Izz_water = Ixx / 2
Iyy_water = Iyy / 10 # much less added water mass in the roll direction

Seabotix_Max_Thrust = 18.0 # N, used in Force_Per_Unit_Thrust only
Force_Per_Unit_Vector_Thrust = Seabotix_Max_Thrust / 127.0 # N
Force_Per_Unit_Prop_Thrust = 2 * Seabotix_Max_Thrust / 127.0 # N
Max_Yaw_Moment   = abs(HBow_At[1] - HStern_At[1]) * Seabotix_Max_Thrust*2
Max_Pitch_Moment = abs(VBow_At[1] - VStern_At[1]) * Seabotix_Max_Thrust*2
Max_Roll_Moment  = Max_Yaw_Moment / 5

# Hydrodynamic model:
# Drag: Newtons per metre per second, modelled from measured terminal velocity:
# (this is a simple exponential model)

Drag_F = np.array((Seabotix_Max_Thrust*2 / 1.0, # x (sideways)
                   Seabotix_Max_Thrust*2 / 2.0, # y (forwards)
                   Seabotix_Max_Thrust*2 / 0.6)) # z (up/down)
# Drag Torque: Newton metres per radian per second, measured from complete
# guesses at maximum rotation rates:
Drag_J = np.array((Max_Yaw_Moment / 1.6,   # yaw
                   Max_Roll_Moment / 2,  # roll
                   Max_Pitch_Moment / 1.6)) # pitch

class Model(base_model.Model):
    def __init__(self, profile=False, currentx=0, currenty=0, currentz=0):
        self.tzero = None
        self.last_t = self.relativeTime()
        self.last_state_sent = self.relativeTime()
	self.current = np.array((currentx, currenty, currentz))
        base_model.Model.__init__(self, profile=profile)
        self.depth_pub = rospy.Publisher("sim/depth", std_msgs.Float32)
        self.attitude_pub = rospy.Publisher("sim/attitude", ctrl_msgs.Attitude)

    def relativeTime(self):
        '''return relative time in floating point seconds since tzero'''
        if self.tzero is None:
            self.tzero = datetime.datetime.now()
        return tdToFloatSeconds(datetime.datetime.now() - self.tzero)

    def processUpdate(self, s):
        # state is stored in:
        # self.displacement = (x,y,z)   - global coordinates
        # self.velocity = (x,y,z)       - global coordinates
        # self.orientation = Quat(...)  - of vehicle relative to the
        #                                 world x=East,y=North,z=Up
        #                                 coords
        #   a_local_vec = orientation.inverse().rotate(global vec)
        #   a_global_vec = orientation.rotate(local vec)
        # self.angular_velocity = (dYaw/dt, dRoll/dt, dPitch/dt)
        #
        # yaw is rotation about the z axis
        # pitch is rotation about the x axis
        # roll is rotation about the y axis
        #
        weight_vec = np.array((0,0,-1))
        buoyancy_vec = np.array((0,0,1))
        #debug(str(s))

        now = self.relativeTime()
        dt = now - self.last_t
        if dt > 10:
            warning('processUpdate called too infrequently! Maximum time step will be 10 seconds.')
            dt = 10
        if dt < 0:
            warning('Clock skew!')
            if dt < -10:
                error('FATAL: Clock skewed by more than 10 seconds!')
                self.stop()
                return

        # Update velocity with forces:
        
        # Forces in vehicle-local coordinates:
        hbow_force   = HBow_Vec * s.HBow * Force_Per_Unit_Vector_Thrust
        vbow_force   = VBow_Vec * s.VBow * Force_Per_Unit_Vector_Thrust
        hstern_force = HStern_Vec * s.HStern * Force_Per_Unit_Vector_Thrust
        vstern_force = VStern_Vec * s.VStern * Force_Per_Unit_Vector_Thrust
        prop_force   = Prop_Vec * s.Prop * Force_Per_Unit_Prop_Thrust
        drag_force   = -Drag_F * self.orientation.inverse().rotate((self.velocity)-(self.current));
        local_force = sum(
            (hbow_force, vbow_force, hstern_force, vstern_force, prop_force, drag_force)
        )

        #debug('local force (R,F,U) = %s' % local_force)
        
        # now in global coordinates:
        global_force = self.orientation.rotate(local_force, mkVec)
        #debug('global force (E,N,U) = %s' % global_force)

        weight_force = weight_vec * Mass * 9.81
        buoyancy_force = buoyancy_vec * Displacement * 9.81
        if self.displacement[2] > 0: 
            p = min(self.displacement[2], 0.4)
            debug('auv appears to be above the surface', 3)
            buoyancy_force -= buoyancy_force * (p/0.4)

        force = sum((global_force, weight_force, buoyancy_force))

        self.velocity += force * dt / Mass

        # and angular velocities with moments:
        # these moments are (about x axis, about y axis, about z axis)
        # in vehicle-local coordinates:
        hbow_moment   = np.cross(HBow_At, hbow_force)
        vbow_moment   = np.cross(VBow_At, vbow_force)
        hstern_moment = np.cross(HStern_At, hstern_force)
        vstern_moment = np.cross(VStern_At, vstern_force)
        prop_moment   = np.cross(Prop_At, prop_force) # expect this to be zero!
        weight_local_force   = self.orientation.inverse().rotate(weight_force, mkVec)
        buoyancy_local_force = self.orientation.inverse().rotate(buoyancy_force, mkVec)
        weight_moment = np.cross(Weight_At, weight_local_force)
        buoyancy_moment = np.cross(Buoyancy_At, buoyancy_local_force)
        
        local_moment = sum(
            (hbow_moment, vbow_moment,
             hstern_moment, vstern_moment,
             prop_moment,
             weight_moment,
             buoyancy_moment)
        )
        #debug('local moment = %s' % local_moment)

        # drag moments:
        drag_moment = -Drag_J * self.angular_velocity

        moment = sum((local_moment, drag_moment))

        # apply moment to angular velocity:
        domega = moment * dt / np.array((Ixx + Ixx_water, Iyy + Iyy_water, Izz + Izz_water))
        self.angular_velocity += domega

        # Update positions with velocities:
        self.displacement += self.velocity * dt
        dorientation = Quaternion.fromEuler(self.angular_velocity * dt)
        self.orientation *= dorientation

        self.last_t = now

        # last thing: renormalise orientation:
        if self.orientation.sxx() > 1.0001 or self.orientation.sxx() < 0.9999:
            debug('orientation quat denormalised: it will be renormalised')
            self.orientation = self.orientation.normalised()
        
        # last last thing: send messages reflecting the new state: this has to
        # be rate-limited since it can cause more motor state message to be
        # sent from control - leading to a tight loop
        if self.relativeTime() - self.last_state_sent > 0.05:
            self.sendStateMessages()
            self.last_state_sent = self.relativeTime()

    def sendStateMessages(self):
        # send:
        #   ForePressureMessage
        #   AftPressureMessage
        #   StateMessage
        # 
        # These should match the DepthCalibration message used:
        # Barracuda pressure
        #
        orientation = base_model.orientationToYPR(self.orientation)
        orientation.yaw += random.gauss(0, 0.5)
        orientation.pitch += random.gauss(0, 0.5)
        orientation.roll += random.gauss(0, 0.5)
        self.depth_pub.publish(std_msgs.Float32(-self.displacement[2] + random.gauss(0, 0.02)))

        self.attitude_pub.publish(orientation)
