#!/usr/bin/env python2.7

# Standard Library Modules
import operator
import datetime
import math
import random

# Third Party Modules
import numpy as np # BSD-type license

# CAUV Modules
import base_model
from base_model import calculateRoll, mkVec
from utils.hacks import tdToFloatSeconds
from utils.quaternion import Quaternion
import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info


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
Mass      = 25.0      # kg
Displacement = 25.0   # kg
Weight_At = np.array((0,0,-0.1))
Buoyancy_At = np.array((0,0,0.05))
Length    = 1.4       # length in m, used for moment calculation only
Ixx = Izz = (Mass*(Length**2)/12.0) # kg m^2 (a uniform rod)
Iyy = Ixx / 4
Ixx_water = Izz_water = Ixx / 2
Iyy_water = Iyy / 10 # much less added water mass in the roll direction

Seabotix_Max_Thrust = 11.0 # N, used in Force_Per_Unit_Thrust only
Force_Per_Unit_Vector_Thrust = Seabotix_Max_Thrust / 127.0 # N
Force_Per_Unit_Prop_Thrust = 2 * Seabotix_Max_Thrust / 127.0 # N
Max_Yaw_Moment   = 1.5*Seabotix_Max_Thrust*2
Max_Pitch_Moment = Max_Yaw_Moment
Max_Roll_Moment  = Max_Yaw_Moment / 20

# Hydrodynamic model:
# Drag: Newtons per metre per second, modelled from measured terminal velocity:
# (this is a simple exponential model)
Drag_F = np.array((Seabotix_Max_Thrust*2 / 0.3, # x (sideways)
                   Seabotix_Max_Thrust   / 0.4, # y (forwards)
                   Seabotix_Max_Thrust*2 / 0.7)) # z (up/down)
# Drag Torque: Newton metres per radian per second, measured from complete
# guesses at maximum rotation rates:
Drag_J = np.array((Max_Yaw_Moment / 0.8,   # yaw
                   Max_Roll_Moment / 1,  # roll
                   Max_Pitch_Moment / 0.8)) # pitch

class Model(base_model.Model):
    def __init__(self, node, profile=False):
        self.tzero = None
        self.last_t = self.relativeTime()
        self.last_state_sent = self.relativeTime()
        base_model.Model.__init__(self, node, profile=profile)

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
        drag_force   = -Drag_F * self.orientation.inverse().rotate(self.velocity);

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

    def addPressureNoise(self, pressure):
        return int(pressure + random.gauss(0, 10))
    
    def addOrientationNoise(self, ypr):
        return messaging.floatYPR(
            ypr.yaw + random.gauss(0, 0.5),
            ypr.pitch + random.gauss(0, 0.5),
            ypr.roll + random.gauss(0, 0.5)
        )

    def sendStateMessages(self):
        # send:
        #   ForePressureMessage
        #   AftPressureMessage
        #   StateMessage
        # 
        # These should match the DepthCalibration message used:
        # Barracuda pressure
        depth_offset = -9.48232
        depth_mult = 0.010395
        depth = float(-self.displacement[2])
        pressure = int((depth - depth_offset) / depth_mult)
        # if somehow we're floating above the surface...
        if pressure < 0:
            pressure = 0
        
        orientation = base_model.orientationToYPR(self.orientation)
        
        # make sure control can deal with pressure message ordering
        if random.randint(0,1) == 1:
            self.node.send(messaging.ForePressureMessage(self.addPressureNoise(pressure)))
            self.node.send(messaging.AftPressureMessage(self.addPressureNoise(pressure)))
        else:
            self.node.send(messaging.AftPressureMessage(self.addPressureNoise(pressure)))
            self.node.send(messaging.ForePressureMessage(self.addPressureNoise(pressure)))
        self.node.send(messaging.StateMessage(self.addOrientationNoise(orientation)))




