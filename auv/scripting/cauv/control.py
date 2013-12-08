''' This module defines the AUV class used to control the motors and control loops.'''

import threading
import time

from cauv.debug import info, warning, error, debug

import cauv_control.msg as ctrl_msgs
import std_msgs.msg as std_msgs
import rospy

class AUV_readonly(object):
    '''Provides access to telemetry data'''
    def __init__(self):
        self.current_bearing = None
        self.current_depth = None
        self.current_pitch = None
        rospy.Subscriber("control/attitude", ctrl_msgs.Attitude, self.onAttitude)
        rospy.Subscriber("control/depth", std_msgs.Float32, self.onDepth)

    def getBearing(self):
        '''Return the current bearing: may return None!'''
        return self.current_bearing
    
    def getDepth(self):
        '''Return the current depth: may return None!'''
        return self.current_depth

    def getPitch(self):
        '''Return the current pitch: may return None!'''
        return self.current_pitch
    
    def onAttitude(self, m):
        self.current_bearing = m.yaw
        self.current_pitch = m.pitch

    def onDepth(self, m):
        self.current_depth = m.data

class AUV(AUV_readonly):
    '''The AUV class provides an interface to control the AUV's control loops, motors, and other hardware.'''
    def __init__(self, priority = 0, timeout = 10):
        AUV_readonly.__init__(self)
        self.bearingCV = threading.Condition()
        self.depthCV = threading.Condition()
        self.pitchCV = threading.Condition()
        self.priority = 0
        self.token = int(time.time() * 1000) & 0xffffffff
        info("Control Token: {}".format(self.token))
        self.timeout = timeout #seconds

        self.depth_params_pub   = rospy.Publisher("control/depth/params",    ctrl_msgs.PIDParams, latch = True)
        self.pitch_params_pub   = rospy.Publisher("control/pitch/params",    ctrl_msgs.PIDParams, latch = True)
        self.bearing_params_pub = rospy.Publisher("control/bearing/params",  ctrl_msgs.PIDParams, latch = True)
        self.depth_target_pub   = rospy.Publisher("control/depth/target",    ctrl_msgs.PIDTarget, tcp_nodelay = True)
        self.pitch_target_pub   = rospy.Publisher("control/pitch/target",    ctrl_msgs.PIDTarget, tcp_nodelay = True)
        self.bearing_target_pub = rospy.Publisher("control/bearing/target",  ctrl_msgs.PIDTarget, tcp_nodelay = True)
        self.motor_demand_pub   = rospy.Publisher("control/external_demand", ctrl_msgs.MotorDemand, tcp_nodelay = True)
        self.depth_cal_aft_pub  = rospy.Publisher("control/depth/aft_calibration", ctrl_msgs.DepthCalibration)
        self.depth_cal_fore_pub = rospy.Publisher("control/depth/fore_calibration", ctrl_msgs.DepthCalibration)

        self.motors = ctrl_msgs.MotorDemand()

    def get_token(self):
        return ctrl_msgs.ControlToken(self.token, self.priority, self.timeout * 1000)

    def send_motors(self):
        self.motor_demand_pub.publish(self.motors)

    def kill(self):
        '''Stop all motors, deactivate the control loops.'''
        self.stop()
        self.bearing(None)
        self.pitch(None)
        self.depth(None)
        
    def stop(self):
        '''Stop all motors, but leave the control loops.'''
        self.motors.prop = 0
        self.motors.hbow = 0
        self.motors.vbow = 0
        self.motors.hstern = 0
        self.motors.vstern = 0
        self.send_motors()

    def bearing(self, bearing):
        '''Set a bearing (degrees CW from north) and activate the bearing control loop, or deactivate bearing control if 'None' is passed.'''
        if bearing is not None:
            self.bearing_target_pub.publish(ctrl_msgs.PIDTarget(self.get_token(), True, bearing))
        else:
            self.bearing_target_pub.publish(self.get_token(), False, 0)

    # additional rate parameter (degrees per second) would ideally be
    # implemented by the control loops
    def bearingAndWait(self, bearing, epsilon = 5, timeout = 30, rate=90):
        '''Set a bearing (degrees CW from north) and activate the bearing control loop,
           do not return until the bearing is achieved within 'epsilon' degrees.
           Limit the rate of rotation to 'rate' degrees per second'''
        if bearing is None:
            raise ValueError("Cannot wait for 'None' bearing.")
        startTime = time.time()
        if rate < 90:
            if self.current_bearing is None:
                start_t = time.time()
                self.bearingCV.acquire()
                self.bearingCV.wait(timeout)
                self.bearingCV.release()
                timeout -= (time.time() - start_t)
                if self.current_bearing is None:
                    return self.bearingAndWait(bearing, epsilon, timeout, rate=90)
            delta_up = (bearing - self.current_bearing) % 360
            delta_down = (self.current_bearing - bearing) % 360
            if delta_up < delta_down:
                delta = delta_up
            elif delta_down < delta_up:
                delta = -delta_down
            debug("%s %s" % (delta_down, delta_up))
            x = 0
            for x in xrange(1, int(abs(delta) / rate)):
                debug(str(x))
                self.bearing(self.current_bearing + x * delta);
                time.sleep(1.0)
            return self.bearingAndWait(bearing, epsilon, timeout=max(1,timeout-x), rate=90)
        else:
            self.bearing(bearing)
            while time.time() - startTime < timeout:
                if self.current_bearing == None or min((bearing - self.current_bearing) % 360, (self.current_bearing - bearing) % 360) > epsilon:
                    #print 'bearing waiting'
                    self.bearingCV.acquire()
                    self.bearingCV.wait(timeout - time.time() + startTime)
                    self.bearingCV.release()
                else:
                    break

    #The values of fore multiplier need to be copied into simulation to work
    def calibrateDepth(self, foreOffset, foreMultiplier, aftOffset=None, aftMultiplier=None):
        '''Set the conversion factors between pressure sensor values and depth.
            
            See also:
                calibrateForSaltWater()
                calibrateForFreshWater()
                autoCalibrateDepth()
        '''
        if aftOffset is None:
            aftOffset = foreOffset
            if aftMultiplier is None:
                aftMultiplier = foreMultiplier
            else:
                warning("Warning: aftMultiplier set but aftOffset not set -- using aftOffset := foreOffset")
        elif aftMultiplier is None:
            warning("Warning: aftOffset set but aftMultiplier not set -- using aftMultiplier := foreMultiplier")

        self.depth_cal_aft_pub.publish (ctrl_msgs.DepthCalibrationMessage(aftOffset,  aftMultiplier))
        self.depth_cal_fore_pub.publish(ctrl_msgs.DepthCalibrationMessage(foreOffset, foreMultiplier))

    def calibrateForSaltWater(self):
        '''Redherring calibration: Set the depth calibration for seawater.'''
        self.calibrateDepth(-912.2/96.2, 1.0/96.2)

    def calibrateForFreshWater(self):
        '''Redherring calibration: Set the depth calibration for fresh water.'''
        self.calibrateDepth(-928.0/86.5, 1.0/86.5)
        
    def autoCalibrateDepth(self, surfacePressure = 1015.0, waterDensity = 1025.0):
        '''Barracuda calibration: Set the depth calibration for barracuda.'''
        # pressure = (depth - fore_offset) / fore_mult
        #    depth = (pressure*fore_mult)+fore_offset
        #
        # pressure = surfacePressure + waterDensity*g*depth
        #          = (depth + surfacePressure)/(waterDensity*g)) * (waterDensity*g)
        #
        # This is pressure in Pa, but we use pressure in mbar, so scale by 100
        
        fore_mult = 1/(waterDensity*9.81)
        fore_offset = -surfacePressure*100 * fore_mult
        
        self.calibrateDepth(fore_offset, fore_mult*100)

    def depth(self, depth):
        '''Set a depth (metres) and activate the depth control loop, or deactivate depth control if 'None' is passed.'''

        if depth is not None:
            self.depth_target_pub.publish(ctrl_msgs.PIDTarget(self.get_token(), True, depth))
        else:
            self.depth_target_pub.publish(ctrl_msgs.PIDTarget(self.get_token(), False, 0))

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
            self.pitch_target_pub.publish(ctrl_msgs.PIDTarget(self.get_token(), True, pitch))
        else:
            self.pitch_target_pub.publish(ctrl_msgs.PIDTarget(self.get_token(), False, 0))

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

    PID_params_docstring = '''
      See the relevant persist files for the values normally used.

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

    def bearingParams(self, kp, ki, kd, scale=1.0, Ap=1, Ai=1, Ad=1, thr=1, maxError=1000):
        '''Set the bearing control loop parameters.

            This is a sensible example:
                bearingParams(3.5, 0, 35, -1, 1.3, 1.3, 1, 1, 150)
        '''

        self.bearing_params_pub.publish(ctrl_msgs.PIDParams(kp, ki, kd, scale, Ap, Ai, Ad, thr, maxError))

    bearingParams.__doc__ += PID_params_docstring

    def depthParams(self, kp, ki, kd, scale=1.0, Ap=1, Ai=1, Ad=1, thr=1, maxError=1000):
        '''Set the depth control loop parameters.
            
            This is a sensible example:
                depthParams(40, 0.6, 500, 1, 2, 2, 2, 1, 40)
        '''
        self.depth_params_pub.publish(ctrl_msgs.PIDParams(kp, ki, kd, scale, Ap, Ai, Ad, thr, maxError))

    depthParams.__doc__ += PID_params_docstring

    def pitchParams(self, kp, ki, kd, scale=1.0, Ap=1, Ai=1, Ad=1, thr=1, maxError=1000):
        '''Set the pitch control loop parameters.
            
            This is a sensible example:
                pitchParams(0.5, 0.1, 10, 1, 1, 1, 1, 1, 5)
        '''
        self.pitch_params_pub.publish(ctrl_msgs.PIDParams(kp, ki, kd, scale, Ap, Ai, Ad, thr, maxError))

    pitchParams.__doc__ += PID_params_docstring

    def prop(self, value):
        '''Set the prop speed. Positive is forwards, range -127:127'''
        debug("cauv.control.Node.prop(%s)" % value, 5)
        self.checkRange(value)
        self.motors.prop = value
        self.send_motors()

    def hbow(self, value):
        '''Set the horizontal bow thruster speed. Positive is ????, range -127:127'''
        self.checkRange(value)
        self.motors.hbow = value
        self.send_motors()
   
    def vbow(self, value):
        '''Set the vertical bow thruster speed. Positive is ????, range -127:127'''
        self.checkRange(value)
        self.motors.vbow = value
        self.send_motors()

    def hstern(self, value):
        '''Set the horizontal stern thruster speed. Positive is ????, range -127:127'''
        self.checkRange(value)
        self.motors.hstern = value
        self.send_motors()

    def vstern(self, value):
        '''Set the vertical stern thruster speed. Positive is ????, range -127:127'''
        self.checkRange(value)
        self.motors.vstern = value
        self.send_motors()

    def vert(self, value):
        '''Go vertically with a given prop value'''
        self.checkRange(value)
        self.motors.vstern = value
        self.motors.vbow = value
        self.send_motors()

    def strafe(self, value):
        '''go sideways with a given prop value'''
        self.checkRange(value)
        self.motors.hbow = value
        self.motors.hstern = value
        self.send_motors()

    def rotate(self, value):
        '''Rotate with a given prop value'''
        self.checkRange(value)
        self.motors.hbow = value
        self.motors.hstern = -value
        self.send_motors()

    def checkRange(self, value):
        if value < -127 or value > 127:
            raise ValueError("invalid motor value: %d" % value)
    
    def onAttitude(self, m):
        AUV_readonly.onAttitude(self, m)
        self.bearingCV.acquire()
        self.pitchCV.acquire()
        self.bearingCV.notifyAll()
        self.pitchCV.notifyAll()
        self.bearingCV.release()
        self.pitchCV.release()

    def onDepth(self, m):
        AUV_readonly.onDepth(self, m)
        self.depthCV.acquire()
        self.depthCV.notifyAll()
        self.depthCV.release()

