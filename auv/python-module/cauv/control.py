''' This module defines the AUV class used to control the motors and control loops.'''

import messaging
import threading
import time

from cauv.debug import info, warning, error, debug

class AUV(messaging.MessageObserver):
    '''The AUV class provides an interface to control the AUV's control loops, motors, and other hardware.'''
    def __init__(self, node):
        messaging.MessageObserver.__init__(self)
        self.__node = node
        node.addObserver(self)
        node.subMessage(messaging.TelemetryMessage())
        self.current_bearing = None
        self.current_depth = None
        self.current_pitch = None
        self.depth_disabled = True
        self.bearingCV = threading.Condition()
        self.depthCV = threading.Condition()
        self.pitchCV = threading.Condition()

    def send(self, msg):
        '''Send a message to the control group.'''
        self.__node.send(msg, "control")

    def stop(self):
        '''Stop all motors, deactivate the control loops.'''
        self.prop(0)
        self.hbow(0)
        self.vbow(0)
        self.hstern(0)
        self.vstern(0)
        self.bearing(None)
        self.pitch(None)
        self.depth(None)
    
    def getBearing(self):
        '''Return the current bearing: may return None!'''
        return self.current_bearing
    
    def getDepth(self):
        '''Return the current depth: may return None!'''
        return self.current_depth

    def getPitch(self):
        '''Return the current pitch: may return None!'''
        return self.current_pitch

    def bearing(self, bearing):
        '''Set a bearing (degrees CW from north) and activate the bearing control loop, or deactivate bearing control if 'None' is passed.'''
        if bearing is not None:
            self.send(messaging.BearingAutopilotEnabledMessage(True, bearing))
        else:
            self.send(messaging.BearingAutopilotEnabledMessage(False, 0))

    def bearingAndWait(self, bearing, epsilon = 5, timeout = 30):
        '''Set a bearing (degrees CW from north) and activate the bearing control loop, do not return until the bearing is achieved within 'epsilon' degrees'''
        if bearing is None:
            raise ValueError("Cannot wait for 'None' bearing.")
        startTime = time.time()
        self.bearing(bearing)
        while time.time() - startTime < timeout:
            if self.current_bearing == None or min((bearing - self.current_bearing) % 360, (self.current_bearing - bearing) % 360) > epsilon:
                #print 'bearing waiting'
                self.bearingCV.acquire()
                self.bearingCV.wait(timeout - time.time() + startTime)
                self.bearingCV.release()
            else:
                break

    def calibrateDepth(self, foreOffset, foreMultiplier, aftOffset=None, aftMultiplier=None):
        '''Set the conversion factors between pressure sensor values and depth.
            
            See also:
                calibrateForSaltWater()
                calibrateForFreshWater()
        '''
        if aftOffset is None:
            aftOffset = foreOffset
            if aftMultiplier is None:
                aftMultiplier = foreMultiplier
            else:
                warning("Warning: aftMultiplier set but aftOffset not set -- using aftOffset := foreOffset")
        elif aftMultiplier is None:
            warning("Warning: aftOffset set but aftMultiplier not set -- using aftMultiplier := foreMultiplier")

        self.send(messaging.DepthCalibrationMessage(foreOffset, foreMultiplier, aftOffset, aftMultiplier))

    def calibrateForSaltWater(self):
        '''Set the depth calibration for seawater.'''
        self.calibrateDepth(-912.2/96.2, 1.0/96.2)

    def calibrateForFreshWater(self):
        '''Set the depth calibration for fresh water.'''
        self.calibrateDepth(-928.0/86.5, 1.0/86.5)

    def depth(self, depth):
        '''Set a depth (metres) and activate the depth control loop, or deactivate depth control if 'None' is passed.'''
        if depth is not None and not self.depth_disabled:
            self.send(messaging.DepthAutopilotEnabledMessage(True, depth))
        else:
            self.send(messaging.DepthAutopilotEnabledMessage(False, 0))

    def depthAndWait(self, depth, epsilon = 0.25, timeout = 30):
        '''Set a depth (metres) and activate the depth control loop, do not return until the depth is achieved within 'epsilon' metres'''
        startTime = time.time()
        self.depth(depth)
        while time.time() - startTime < timeout:
            if self.current_depth == None or abs(depth - self.current_depth) > epsilon:
                self.depthCV.acquire()
                self.depthCV.wait(timeout - time.time() + startTime)
                self.depthCV.release()
            else:
                break

    def pitch(self, pitch):
        '''Set a pitch (degrees up) and activate the pitch control loop, or deactivate pitch control if 'None' is passed.'''
        if pitch is not None:
            self.send(messaging.PitchAutopilotEnabledMessage(True, pitch))
        else:
            self.send(messaging.PitchAutopilotEnabledMessage(False, 0))

    def pitchAndWait(self, pitch, epsilon = 5, timeout = 30):
        '''Set a pitch (degrees up) and activate the pitch control loop, do not return until the pitch is achieved within 'epsilon' degrees'''
        startTime = time.time()
        self.pitch(pitch)
        while time.time() - startTime < timeout:
            if self.current_pitch == None or min((pitch - self.current_pitch) % 360, (self.current_pitch - pitch) % 360) > epsilon:
                self.pitchCV.acquire()
                self.pitchCV.wait(timeout - time.time() + startTime)
                self.pitchCV.release()
            else:
                break

    def bearingParams(self, kp, ki, kd, scale=1.0, Ap=1, Ai=1, Ad=1, thr=1, maxError=1000):
        '''Set the bearing control loop parameters.

            This is a sensible example:
                bearingParams(3.5, 0, 35, -1, 1.3, 1.3, 1, 1, 150)
            
            Basic parameters:

            kp    : proportional control constant
            ki    : integral control constant
            kd    : derivative control constant
            
            More parameters:

            scale : scale applied to all parameters (normally 1)
            Ap    : adaptive control: proportional control is allowed to vary
                    between kp/Ap and kp*Ap
            Ai    : adaptive control: integral control is allowed to vary
                    between ki/Ai and ki*Ai
            Ad    : adaptive control: derivative control is allowed to vary
                    between kd/Ad and kd*Ad
            thr   : adaptive control: (error/thr) controls the amount kp and ki
                    are increased from their minimum values, and the amount kd
                    is decreased from its maximum value
            maxError : prevent integral wind-up by clamping the inegral error
                       to +- this value. The proportional and derivative errors
                       are also clamped into the range +- maxError

        '''
        self.send(messaging.BearingAutopilotParamsMessage(kp, ki, kd, scale, Ap, Ai, Ad, thr, maxError))

    def depthParams(self, kp, ki, kd, scale=1.0, Ap=1, Ai=1, Ad=1, thr=1, maxError=1000):
        '''Set the depth control loop parameters.
            
            This is a sensible example:
                depthParams(40, 0.6, 500, 1, 2, 2, 2, 1, 40)
                    
            Basic parameters:

            kp    : proportional control constant
            ki    : integral control constant
            kd    : derivative control constant
            
            More parameters:

            scale : scale applied to all parameters (normally 1)
            Ap    : adaptive control: proportional control is allowed to vary
                    between kp/Ap and kp*Ap
            Ai    : adaptive control: integral control is allowed to vary
                    between ki/Ai and ki*Ai
            Ad    : adaptive control: derivative control is allowed to vary
                    between kd/Ad and kd*Ad
            thr   : adaptive control: (error/thr) controls the amount kp and ki
                    are increased from their minimum values, and the amount kd
                    is decreased from its maximum value
            maxError : prevent integral wind-up by clamping the inegral error
                       to +- this value. The proportional and derivative errors
                       are also clamped into the range +- maxError
        '''
        self.send(messaging.DepthAutopilotParamsMessage(kp, ki, kd, scale, Ap, Ai, Ad, thr, maxError))

    def pitchParams(self, kp, ki, kd, scale=1.0, Ap=1, Ai=1, Ad=1, thr=1, maxError=1000):
        '''Set the pitch control loop parameters.
            
            This is a sensible example:
                pitchParams(0.5, 0.1, 10, 1, 1, 1, 1, 1, 5)
             
            Basic parameters:

            kp    : proportional control constant
            ki    : integral control constant
            kd    : derivative control constant
            
            More parameters:

            scale : scale applied to all parameters (normally 1)
            Ap    : adaptive control: proportional control is allowed to vary
                    between kp/Ap and kp*Ap
            Ai    : adaptive control: integral control is allowed to vary
                    between ki/Ai and ki*Ai
            Ad    : adaptive control: derivative control is allowed to vary
                    between kd/Ad and kd*Ad
            thr   : adaptive control: (error/thr) controls the amount kp and ki
                    are increased from their minimum values, and the amount kd
                    is decreased from its maximum value
            maxError : prevent integral wind-up by clamping the inegral error
                       to +- this value. The proportional and derivative errors
                       are also clamped into the range +- maxError
        '''
        self.send(messaging.PitchAutopilotParamsMessage(kp, ki, kd, scale, Ap, Ai, Ad, thr, maxError))

    def prop(self, value):
        '''Set the prop speed. Positive is forwards, range -127:127'''
        debug("cauv.control.Node.prop(%s)" % value, 5)
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.Prop, value))

    def hbow(self, value):
        '''Set the horizontal bow thruster speed. Positive is ????, range -127:127'''
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HBow, value))
   
    def vbow(self, value):
        '''Set the vertical bow thruster speed. Positive is ????, range -127:127'''
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VBow, value))

    def hstern(self, value):
        '''Set the horizontal stern thruster speed. Positive is ????, range -127:127'''
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HStern, value))

    def vstern(self, value):
        '''Set the vertical stern thruster speed. Positive is ????, range -127:127'''
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VStern, value))
        
    def forwardlights(self, value):
        '''Set the forwards light power. Range 0-255'''
        self.checkLightValue(value)
        self.__node.send(messaging.LightMessage(messaging.LightID.Forward, value))
        
    def downlights(self, value):
        '''Set the downwards light power. Range 0-255'''
        self.checkLightValue(value)
        self.__node.send(messaging.LightMessage(messaging.LightID.Down, value))
        
    def checkLightValue(self, value):
        '''Raise an error if 'value' is not in the range accepted for light control.'''
        if not (value>=0 and value<256):
            raise ValueError("invalid light value: %d" % value)

    def cut(self, strength):
        '''Control the cutting device: 1=cut, 0=don't cut.'''
        # strength is 0 = off, 1 = on
        self.send(messaging.CuttingDeviceMessage(strength))
    
    def motorMap(self, motor_id, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        '''Apply linear mapping to motor power values.
        
           map:
                    -127                           0                            127
             demand:  |---------------------------|0|----------------------------| 
             output:     |---------------|         0            |----------------|
                         ^               ^                      ^                ^
                         maxMinus     zeroMinus               zeroPlus        maxPlus

        '''
        m = messaging.MotorMap()
        m.zeroPlus = zero_plus
        m.zeroMinus = zero_minus
        m.maxPlus = max_plus
        m.maxMinus = max_minus
        self.send(messaging.SetMotorMapMessage(motor_id, m))

    def propMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        '''see doc for motorMap'''
        self.motorMap(messaging.MotorID.Prop, zero_plus, zero_minus, max_plus, max_minus)

    def hbowMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        '''see doc for motorMap'''
        self.motorMap(messaging.MotorID.HBow, zero_plus, zero_minus, max_plus, max_minus)

    def vbowMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        '''see doc for motorMap'''
        self.motorMap(messaging.MotorID.VBow, zero_plus, zero_minus, max_plus, max_minus)

    def vsternMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        '''see doc for motorMap'''
        self.motorMap(messaging.MotorID.VStern, zero_plus, zero_minus, max_plus, max_minus)

    def hsternMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        '''see doc for motorMap'''
        self.motorMap(messaging.MotorID.HStern, zero_plus, zero_minus, max_plus, max_minus)

    def v(self, value):
        self.checkRange(value)
        self.vbow(value)
        self.vstern(value)

    def strafe(self, value):
        debug("cauv.control.Node.strafe(%s)" % value, 5)
        self.checkRange(value)
        self.hbow(value)
        self.hstern(value)

    def r(self, value):
        self.checkRange(value)
        self.hbow(value)
        self.hstern(-value)

    def checkRange(self, value):
        if value < -127 or value > 127:
            raise ValueError("invalid motor value: %d" % value)
    
    def onTelemetryMessage(self, m):
        self.current_bearing = m.orientation.yaw
        self.current_depth = m.depth
        self.current_pitch = m.orientation.pitch
        self.bearingCV.acquire()
        self.depthCV.acquire()
        self.pitchCV.acquire()
        self.bearingCV.notifyAll()
        self.depthCV.notifyAll()
        self.pitchCV.notifyAll()
        self.bearingCV.release()
        self.depthCV.release()
        self.pitchCV.release()

