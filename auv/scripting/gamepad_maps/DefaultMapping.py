#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info

from gamepad_maps.GamepadMapping import *

class ConcreteGamepadMapping(GamepadMapping):
    def __init__(self, auv, xbox_axes, xbox_buttons):
        GamepadMapping.__init__(self, auv)
        self.Locked = True
        self.JOY_RPressed = False
        self.auv.depth_disabled = False
        self.xbox_axes = xbox_axes
        self.xbox_buttons = xbox_buttons
        
    def controls(self):
        return """
        While holding right trigger (Unlock):
            Left Stick: Prop (Up/Down), Strafe (Left/Right).
            Right Stick: Pitch(Up/Down), Turn (Left/Right), Set Bearing (Left/Right while holding stick in).
            D-Pad: Depth/Bearing in 0.1 m/5 degree increments.
        
        X turns everything off.
        A sets the gamepad to a low priority so AI can override.
        B sets the gamepad to a high priority (the default)
        """
    
    def buttonPressed(self, button):
        if button == self.xbox_buttons.X:
            info("Stop!")
            self.auv.kill()
            self.bearing_state = False
            self.pitch_state = False
            self.depth_state = False

        if button == self.xbox_buttons.A:
            self.auv.priority = -1

        if button == self.xbox_buttons.B:
            self.auv.priority = 10
            
        #all the following controls are default locked
        if self.Locked:
            return
            
        #Fine control of autopilots
        if button == self.xbox_buttons.LEFT:
            #auto pilot mode check first
            if self.bearing_state == False:
                self.bearing = self.auv.current_bearing
                self.bearing_state = True
            self.bearing = (self.bearing - 5)%360
            self.auv.bearing(self.bearing)
        if button == self.xbox_buttons.RIGHT:
            #auto pilot mode check first
            if self.bearing_state == False:
                self.bearing = self.auv.current_bearing
                self.bearing_state = True
            self.bearing = (self.bearing + 5)%360
            self.auv.bearing(self.bearing)
        if button == self.xbox_buttons.UP:
            #auto pilot mode check first
            if self.depth_state == False:
                self.depth = 0
                self.depth_state = True
            self.depth = self.depth - 0.1
            self.auv.depth(self.depth)
        if button == self.xbox_buttons.DOWN:
            #auto pilot mode check first
            if self.depth_state == False:
                self.depth = 0
                self.depth_state = True
            self.depth = self.depth + 0.1
            self.auv.depth(self.depth)
                        
        if button == self.xbox_buttons.JOY_R:
            self.JOY_RPressed = True
        
    def buttonReleased(self, button):
        if button == self.xbox_buttons.JOY_R:
            self.JOY_RPressed = False
        
    def handleAxis(self, axisId, value):
        if axisId == self.xbox_axes.RT:
            if value > 0.5:
                self.Locked = False
            else:
                self.Locked = True
                self.auv.strafe(0)
                self.auv.prop(0)
                if self.depth_state:
                    self.auv.depth(self.depth)
                if self.bearing_state:
                    self.auv.bearing(self.bearing)
                if self.pitch_state:
                    self.auv.pitch(self.pitch)
        
        #all the following controls are default locked
        if self.Locked:
            return
        if axisId == self.xbox_axes.JOY_R_X:
            #auto pilot mode check first
            if not self.bearing_state:
                self.bearing = self.auv.current_bearing
                self.bearing_state = True
            if value == 0 and not self.JOY_RPressed:
                self.bearing = self.auv.current_bearing
                self.auv.bearing(self.bearing)
            else:
                self.bearing = (self.bearing + (2 * value * value * value))%360
                self.auv.bearing(self.bearing)
        
        if axisId == self.xbox_axes.JOY_R_Y:
            #auto pilot mode check first
            if not self.pitch_state:
                self.pitch = 0
                self.pitch_state = True
            self.pitch = ((self.pitch + (value * value * value) + 180)%360) - 180
            self.auv.pitch(self.pitch)

        if axisId == self.xbox_axes.JOY_L_X:
            self.auv.strafe(-int(value*127))
        
        if axisId == self.xbox_axes.JOY_L_Y:
            self.auv.prop(-int(value*127))
            
    def axisNeutralValue(self, axisId):
        if axisId == self.xbox_axes.LT or axisId == self.xbox_axes.RT:
            return -1.0
        return 0.0
