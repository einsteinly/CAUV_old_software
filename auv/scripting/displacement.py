#TODO check threading
import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info

import time
import threading
import math

# forward/reverse speed scale and exponential factors
defaultMotorSpeedCurves = {
        messaging.MotorID.Prop: 0.5,
        messaging.MotorID.HBow: 1.0,
        messaging.MotorID.HStern: 1.0
        }

defaultMotorSpeedScales = {
        messaging.MotorID.Prop: 0.4 / 127,
        messaging.MotorID.HBow: 0.3 / 127,
        messaging.MotorID.HStern: 0.3 / 127
        }

class Motors:
    def __init__(self, speedScale, speedCurve):
        self.speedRequest = 0
        self.timeLast = time.time()
        self.speedScale = speedScale
        self.speedCurve = speedCurve
        self.speedInMperS = 0
    def updateSpeedInMperS(self, mode='simple'):
        if mode == 'simple':
            self.speedInMperS = self.speedRequest * self.speedScale
        else:
            oldTime = self.timeLast
            self.timeLast = time.time()
            timeDiff = self.timeLast - oldTime
            self.speedInMperS += (self.speedRequest * self.speedScale -
                    self.speedInMperS) * (1 - math.exp(-self.speedCurve * timeDiff))

class Displacement:
    def __init__(self, speedScales = defaultMotorSpeedScales, speedCurves =
            defaultMotorSpeedCurves, mode = 'simple'):
        self.displacementNorth = 0
        self.displacementEast = 0
        self.timeLast = time.time()
        self.update = threading.Condition()
        self.motors = {}
        self.bearing = 0
        self.gotBearing = False
        self.motorsRunning = False
        self.mode = mode
        for motor in speedScales.iterkeys():
            self.motors[motor] = Motors(speedScales[motor], speedCurves[motor])
        t = threading.Thread(target=self.updateDisplacement)
        t.daemon = True
        t.start()

    def getDisplacement(self):
        with self.update:
            r = (self.displacementNorth, self.displacementEast)
        return r

    def resetDisplacement(self):
        with self.update:
            self.displacementNorth = 0
            self.displacementEast = 0

    def motorsRunningUpdate(self):
        self.motorsRunning = False
        for motor, properties in self.motors.iteritems():
            if properties.speedRequest != 0:
                self.motorsRunning = True
                break

    def updateDisplacement(self):
        while True:
            time.sleep(0.25)
            self.motorsRunningUpdate()
            if self.motorsRunning and self.gotBearing:
                with self.update:
                    northerness = math.cos(math.radians(self.bearing))
                    easterness = math.sin(math.radians(self.bearing))
                    oldTime = self.timeLast
                    self.timeLast = time.time()
                    timeDiff = self.timeLast - oldTime
                    for motor, properties in self.motors.iteritems():
                        distance = properties.speedInMperS * timeDiff
                        if motor == messaging.MotorID.Prop:
                            self.displacementNorth += northerness * distance
                            self.displacementEast += easterness * distance
                        elif motor == (messaging.MotorID.HBow or
                                messaging.MotorID.HStern):
                            self.displacementNorth += easterness * distance
                            self.displacementEast += northerness * distance

    def onMotorStateMessage(self, m):
        motor = m.motorID
        if motor in self.motors:
            with self.update:
                self.motors[motor].speedRequest = m.speed
                self.motors[motor].updateSpeedInMperS(self.mode)
        else:
            warning('Motor %s unknown to displacement.py used' % motor)

    def onTelemetryMessage(self, m):
        with self.update:
            self.bearing = m.orientation.yaw#target
            self.gotBearing = True

