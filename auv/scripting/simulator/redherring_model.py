#! /usr/bin/env python

# Standard Library Modules
import operator
import datetime

# Third Party Modules
import Quaternion # BSD licensed: http://pypi.python.org/pypi/Quaternion/
from Quaternion import Quat
import numpy as np # BSD-type license

# CAUV Modules
import base_model
from utils.hacks import tdToFloatSeconds
from cauv.debug import debug, error, warning, info


# Rigid body model:
#
# Plan view:
# ==========
#
# front                                   back
#
#     Y_Bow     origin at CoM
#       |<-------------|
#                     CoM            Y_Stern
#                      |--------------->|
#
#                      x
#                      |
#      hbow            |              hstern
#     ..|..............|................|..
#    <--X-------red-he-o-rring----------X-->y prop
#    ...|...............................|..
#
# Side View:
# ==========
#                      z
#                      |
#      hbow            |              hstern
#     ..|..............|................|..
#    <--X-------red-he-o-rring----------X-->y prop
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
# Position vectors of centre of thrust, and vector of direction of thrust:
# All in m:
HBow_At   = np.array(0,-0.8,0); HBow_Vec   = np.array( 1,0,0)
HStern_At = np.array(0, 0.8,0); HStern_Vec = np.array(-1,0,0) # control flips vector
VBow_At   = np.array(0,-0.8,0); VBow_Vec   = np.array( 0,0,1)
VStern_At = np.array(0, 0.8,0); VStern_Vec = np.array( 0,0,1) # control flips vector
Prop_At   = np.array(0, 0.9,0); Prop_Vec   = np.array( 0,1,0)
Mass      = 37.0      # kg
Weight    = -0.1      # N downwards (i.e., slightly buoyant)
Weight_At = np.array(0,0,0)
Length    = 1.3       # length in m, used for moment calculation only
Ixx = Izz = (Mass*(Length**2)/12.0) # kg m^2
Iyy = Ixx / 4

Max_Thrust = 22.0 # N, used in Force_Per_Unit_Thrust only
Force_Per_Unit_Thrust = Max_Thrust/127.0 # N

# Hydrodynamic model:
# Drag: Newtons per metre per second, modelled from measured terminal velocity:
# (this is a simple exponential model)
Drag_F = np.array(Max_Thrust*2 / 0.3, # x (sideways)
                  Max_Thrust   / 0.4, # y (forwards)
                  Max_Thrust*2 / 0.7) # z (up/down)


def rotate(v, q):
    q = q.q
    t0 =  q[0] * q[1];
    t1 =  q[0] * q[2];
    t2 =  q[0] * q[3];
    t3 = -q[1] * q[1];
    t4 =  q[1] * q[2];
    t5 =  q[1] * q[3];
    t6 = -q[2] * q[2];
    t7 =  q[2] * q[3];
    t8 = -q[3] * q[3];
    d = np.array(
        2*((t6 + t8) * v[0] + (t4 - t2) * v[1] + (t1 + t5) * v[2]),
        2*((t2 + t4) * v[0] + (t3 + t8) * v[1] + (t7 - t0) * v[2]),
        2*((t5 - t1) * v[0] + (t0 + t7) * v[1] + (t3 + t6) * v[2])
    )
    return v + d

class Model(base_model.Model):
    def __init__(self, node):
        base_model.Model.__init__(self, node)
        self.tzero = None
        self.last_t = self.relativeTime()

    def relativeTime(self):
        # return relative time in floating point seconds since tzero
        if self.tzero is None:
            self.tzero = datetime.datetime.now()
        return tdToFloatSeconds(datetime.datetime.now() - self.tzero)

    def processUpdate(self, s):
        # state is stored in:
        # self.displacement = (x,y,z)
        # self.velocity = (x,y,z)
        # self.orientation = Quat(...)
        # self.angular_velocity = (dYaw/dt, dPitch/dt, dRoll/dt)
        #
        # yaw is rotation about the z axis
        # pitch is rotation about the x axis
        # roll is rotation about the y axis
        #
        # To get the current vector (in world coords) from the CoM to one of
        # the centres of thrust it is rotated by the current orientation quat:
        hbow_at   = rotate(HBow_At, self.orientation)
        hstern_at = rotate(HStern_At, self.orientation)
        vbow_at   = rotate(VBow_At, self.orientation)
        vstern_at = rotate(VStern_At, self.orientation)
        prop_at   = rotate(Prop_At, self.orientation)
        weight_at = rotate(Weight_At, self.orientation)
        # And similarly for the directions in which forces act:
        # (except for weight of course)
        hbow_vec   = rotate(HBow_Vec, self.orientation)
        hstern_vec = rotate(HStern_Vec, self.orientation)
        vbow_vec   = rotate(VBow_Vec, self.orientation)
        vstern_vec = rotate(VStern_Vec, self.orientation)
        prop_vec   = rotate(Prop_Vec, self.orientation)
        weight_vec = np.array(0,0,1)

        now = self.relativeTime()
        dt = now - self.last_t

        # Update velocity with forces:

        hbow_force   = hbow_vec * s.HBow * Force_Per_Unit_Thrust
        vbow_force   = vbow_vec * s.VBow * Force_Per_Unit_Thrust
        hstern_force = hstern_vec * s.HStern * Force_Per_Unit_Thrust
        vstern_force = vstern_vec * s.VStern * Force_Per_Unit_Thrust
        prop_force   = prop_vec * s.Prop * Force_Per_Unit_Thrust
        weight_force = weight_vec * Weight
        drag_force   = -Drag_F * self.velocity

        force = sum(
            (hbow_force, vbow_force,
             hstern_force, vstern_force,
             prop_force,
             drag_force,
             weight_force)
        )

        self.velocity += force * dt / Mass

        # and angular velocities with moments:
        # these moments are (about x axis, about y axis, about z axis)
        hbow_moment   = np.cross(hbow_force, hbow_at)
        vbow_moment   = np.cross(vbow_force, vbow_at)
        hstern_moment = np.cross(hstern_force, hstern_at)
        vstern_moment = np.cross(vstern_force, vstern_at)
        prop_moment   = np.cross(prop_force, prop_at) # expect this to be zero!
        weight_moment = np.cross(weight_force, weight_at)

        total_moment = sum(
            (hbow_moment, vbow_moment,
             hstern_moment, vstern_moment,
             prop_moment,
             weight_moment)
        )

        # we're in the global reference frame - convert to vehicle-local frame:
        local_moment = rotate(total_moment, self.orientation.inv())

        # convert moment to (yaw, pitch, roll) = (about z axis, about x axis, about y axis)
        local_moment = np.array(
            local_moment[2], local_moment[0], local_moment[1]
        )

        domega = local_moment * dt / np.array(Izz, Ixx, Iyy)
        self.angular_velocity += domega

        # Update positions with velocities:
        self.displacement += self.velocity * dt
        dorientation = Quat(self.angular_velocity * dt)
        self.orientation += dorientation

        self.last_t = now

        # last thing: renormalise orientation:
        normalised_orientation  = Quaternion.normalize(self.orientation.q)
        if sum((normalised_orientation.equatorial -
                self.orientation.equatorial) ** 2) > 0.0001:
            debug('orientation quat denormalised: it will be renormalised')
            self.orientation = self.normalised_o





