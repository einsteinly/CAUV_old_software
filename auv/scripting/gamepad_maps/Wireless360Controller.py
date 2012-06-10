import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info

from gamepad_maps.GamepadMapping import *

class XBoxButtons:
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3
    START = 4
    BACK = 5
    JOY_L = 6
    JOY_R = 7
    LB = 8
    RB = 9
    XBOX = 10
    A = 11
    B = 12
    X = 13
    Y = 14

class XBoxAxes:
    JOY_L_X = 0
    JOY_L_Y = 1
    JOY_R_X = 2
    JOY_R_Y = 3
    LT = 4
    RT = 5

class ConcreteGamepadMapping(GamepadMapping):
    def __init__(self, auv):
        GamepadMapping.__init__(self, auv)
    
    def buttonPressed(self, button):
        if button == XBoxButtons.A:
            info("Running prop")
            self.auv.prop(127)
        if button == XBoxButtons.X:
            info("Stop!")
            self.auv.stop()
    
    def buttonReleased(self, button):
        if button == XBoxButtons.A:
            info("Stopping prop")
            self.auv.prop(0)

    def handleAxis(self, axisId, value):
        if axisId == XBoxAxes.JOY_R_X:
            self.bearing = (self.bearing + (2 * value * value * value))%360
            self.auv.bearing(self.bearing)
        
        if axisId == XBoxAxes.JOY_R_Y:
            self.pitch = ((self.pitch + (value * value * value) + 180)%360) - 180
            self.auv.pitch(self.pitch)

    def axisNeutralValue(self, axisId):
        if axisId == 4 or axisId == 5:
            return -1.0
        return 0.0
