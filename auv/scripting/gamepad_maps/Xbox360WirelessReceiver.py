import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info

from gamepad_maps.GamepadMapping import *

class XBoxButtons:
    UP = 13
    DOWN = 14
    LEFT = 11
    RIGHT = 12
    START = 7
    BACK = 6
    JOY_L = 9
    JOY_R = 10
    LB = 4
    RB = 5
    XBOX = 8
    A = 0
    B = 1
    X = 2
    Y = 3

class XBoxAxes:
    JOY_L_X = 0
    JOY_L_Y = 1
    JOY_R_X = 3
    JOY_R_Y = 4
    LT = 2
    RT = 5

class ConcreteGamepadMapping(GamepadMapping):
    def __init__(self, auv):
        GamepadMapping.__init__(self, auv)
        self.Locked = True
        self.JOY_RPressed = False
        self.auv.depth_disabled = False
        
    def controls(self):
        return """
        While holding right trigger (Unlock):
            Left Stick: Prop (Up/Down), Strafe (Left/Right).
            Right Stick: Pitch(Up/Down), Turn (Left/Right), Set Bearing (Left/Right while holding stick in).
            D-Pad: Depth/Bearing in 0.1 m/5 degree increments.
        
        X turns everything off.
        """
    
    def buttonPressed(self, button):
        #if button == XBoxButtons.A:
        #    info("Running prop")
        #    self.auv.prop(127)
        if button == XBoxButtons.X:
            info("Stop!")
            self.auv.stop()
            self.bearing_state = False
            self.pitch_state = False
            self.depth_state = False
            
        #all the following controls are default locked
        if self.Locked:
            return
            
        #Fine control of autopilots
        if button == XBoxButtons.LEFT:
            #auto pilot mode check first
            if self.bearing_state == False:
                self.bearing = self.auv.current_bearing
                self.bearing_state = True
            self.bearing = (self.bearing - 5)%360
            self.auv.bearing(self.bearing)
        if button == XBoxButtons.RIGHT:
            #auto pilot mode check first
            if self.bearing_state == False:
                self.bearing = self.auv.current_bearing
                self.bearing_state = True
            self.bearing = (self.bearing + 5)%360
            self.auv.bearing(self.bearing)
        if button == XBoxButtons.UP:
            #auto pilot mode check first
            if self.depth_state == False:
                self.depth = 0
                self.depth_state = True
            self.depth = self.depth - 0.1
            self.auv.depth(self.depth)
        if button == XBoxButtons.DOWN:
            #auto pilot mode check first
            if self.depth_state == False:
                self.depth = 0
                self.depth_state = True
            self.depth = self.depth + 0.1
            self.auv.depth(self.depth)
                        
        if button == XBoxButtons.JOY_R:
            self.JOY_RPressed = True
        
    def buttonReleased(self, button):
        if button == XBoxButtons.JOY_R:
            self.JOY_RPressed = False
        
    def handleAxis(self, axisId, value):
        #all the following controls are default locked
        if axisId == XBoxAxes.RT:
            if value > 0.5:
                self.Locked = False
                info('Unlocked')
            else:
                self.Locked = True
                info('Locked')  
        
        if self.Locked:
            return
        if axisId == XBoxAxes.JOY_R_X:
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
        
        if axisId == XBoxAxes.JOY_R_Y:
            #auto pilot mode check first
            if not self.pitch_state:
                self.pitch = 0
                self.pitch_state = True
            self.pitch = ((self.pitch + (value * value * value) + 180)%360) - 180
            self.auv.pitch(self.pitch)

        if axisId == XBoxAxes.JOY_L_X:
            self.auv.strafe(-int(value*127))
        
        if axisId == XBoxAxes.JOY_L_Y:
            self.auv.prop(-int(value*127))
            
    def axisNeutralValue(self, axisId):
        if axisId == XBoxAxes.LT or axisId == XBoxAxes.RT:
            return -1.0
        return 0.0
