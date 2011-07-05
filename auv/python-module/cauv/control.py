import messaging
import threading
import time

#pylint: disable=E1101

class AUV(messaging.BufferedMessageObserver):
    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("control")
        node.join("telemetry")
        node.addObserver(self)
        self.current_bearing = None
        self.current_depth = None
        self.current_pitch = None
        self.bearingCV = threading.Condition()
        self.depthCV = threading.Condition()
        self.pitchCV = threading.Condition()

    def send(self, msg):
        # send to control via self.__node
        self.__node.send(msg, "control")

    def stop(self):
        self.prop(0)
        self.hbow(0)
        self.vbow(0)
        self.hstern(0)
        self.vstern(0)
        self.bearing(None)
        self.pitch(None)
        self.depth(None)
    
    def getBearing(self):
        return self.current_bearing

    def bearing(self, bearing):
        if bearing is not None:
            self.send(messaging.BearingAutopilotEnabledMessage(True, bearing))
        else:
            self.send(messaging.BearingAutopilotEnabledMessage(False, 0))

    def bearingAndWait(self, bearing, epsilon = 5, timeout = 30):
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
        if aftOffset is None:
            aftOffset = foreOffset
            if aftMultiplier is None:
                aftMultiplier = foreMultiplier
            else:
                print "Warning: aftMultiplier set but aftOffset not set -- using aftOffset := foreOffset"
        elif aftMultiplier is None:
            print "Warning: aftOffset set but aftMultiplier not set -- using aftMultiplier := foreMultiplier"

        self.send(messaging.DepthCalibrationMessage(foreOffset, foreMultiplier, aftOffset, aftMultiplier))

    def calibrateForSaltWater(self):
        self.calibrateDepth(-912.2/96.2, 1.0/96.2)

    def calibrateForFreshWater(self):
        self.calibrateDepth(-928.0/86.5, 1.0/86.5)

    def depth(self, depth):
        if depth is not None:
            self.send(messaging.DepthAutopilotEnabledMessage(True, depth))
        else:
            self.send(messaging.DepthAutopilotEnabledMessage(False, 0))

    def depthAndWait(self, depth, epsilon = 5, timeout = 30):
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
        if pitch is not None:
            self.send(messaging.PitchAutopilotEnabledMessage(True, pitch))
        else:
            self.send(messaging.PitchAutopilotEnabledMessage(False, 0))

    def pitchAndWait(self, pitch, epsilon = 5, timeout = 30):
        startTime = time.time()
        self.pitch(pitch)
        while time.time() - startTime < timeout:
            if self.current_pitch == None or min((pitch - self.current_pitch) % 360, (self.current_pitch - pitch) % 360) > epsilon:
                self.pitchCV.acquire()
                self.pitchCV.wait(timeout - time.time() + startTime)
                self.pitchCV.release()
            else:
                break

    def bearingParams(self, kp, ki, kd, scale, Ap=1, Ai=1, Ad=1, thr=1, maxError=1000):
        self.send(messaging.BearingAutopilotParamsMessage(kp, ki, kd, scale, Ap, Ai, Ad, thr, maxError))

    def depthParams(self, kp, ki, kd, scale, Ap=1, Ai=1, Ad=1, thr=1, maxError=1000):
        self.send(messaging.DepthAutopilotParamsMessage(kp, ki, kd, scale, Ap, Ai, Ad, thr, maxError))

    def pitchParams(self, kp, ki, kd, scale, Ap=1, Ai=1, Ad=1, thr=1, maxError=1000):
        self.send(messaging.PitchAutopilotParamsMessage(kp, ki, kd, scale, Ap, Ai, Ad, thr, maxError))

    def prop(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.Prop, value))

    def hbow(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HBow, value))
   
    def vbow(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VBow, value))

    def hstern(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HStern, value))

    def vstern(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VStern, value))
        
    def forwardlights(self, value):
        self.checkLightValue(value)
        self.__node.send(messaging.LightMessage(messaging.LightID.Forward, value))
        
    def downlights(self, value):
        self.checkLightValue(value)
        self.__node.send(messaging.LightMessage(messaging.LightID.Down, value))
        
    def checkLightValue(self, value):
        if not (value>=0 and value<256):
            raise ValueError("invalid light value: %d" % value)

    def cut(self, strength):
        # strength is 0 = off, 1 = on
        self.send(messaging.CuttingDevice(strength))
    
    def motorMap(self, motor_id, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        #
        #        -127                           0                            127
        # demand:  |---------------------------|0|----------------------------| 
        # output:     |---------------|         0            |----------------|
        #             ^               ^                      ^                ^
        #             maxMinus     zeroMinus               zeroPlus        maxPlus
        #
        m = messaging.MotorMap()
        m.zeroPlus = zero_plus
        m.zeroMinus = zero_minus
        m.maxPlus = max_plus
        m.maxMinus = max_minus
        self.send(messaging.SetMotorMapMessage(motor_id, m))

    def propMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        # see doc for motorMap
        self.motorMap(messaging.MotorID.Prop, zero_plus, zero_minus, max_plus, max_minus)

    def hbowMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        # see doc for motorMap        
        self.motorMap(messaging.MotorID.HBow, zero_plus, zero_minus, max_plus, max_minus)

    def vbowMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        # see doc for motorMap        
        self.motorMap(messaging.MotorID.VBow, zero_plus, zero_minus, max_plus, max_minus)

    def vsternMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        # see doc for motorMap        
        self.motorMap(messaging.MotorID.VStern, zero_plus, zero_minus, max_plus, max_minus)

    def hsternMap(self, zero_plus, zero_minus, max_plus = 127, max_minus = -127):
        # see doc for motorMap        
        self.motorMap(messaging.MotorID.HStern, zero_plus, zero_minus, max_plus, max_minus)

    def v(self, value):
        self.vbow(value)
        self.vstern(value)

    def strafe(self, value):
        self.hbow(value)
        self.hstern(value)

    def r(self, value):
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

