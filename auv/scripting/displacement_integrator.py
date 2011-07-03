import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info

import time
import threading
import math

# forward/reverse speed scale and exponential factors
class settings:
    propSpeedScale   = 0.4 / 127 # metres per second per unit motor power
    propSpeedCurve   =   1 / 2.0 # 1 / time to get to 63% speed going forwards
    strafeSpeedScale = 0.3 / 127 # metres per second per unit motor power
    strafeSpeedCurve =   1 / 1.0 # 1 / time to get to 63% speed strafing

class DisplacementIntegrator:
    def __init__(self):
        self.displacementE = 0
        self.displacementN = 0
        self.displacementZ = 0
        self.timeLast = time.time()
        self.update = threading.Condition()
        self.propSpeedReq = 0
        self.hbowSpeedReq = 0
        self.hsternSpeedReq = 0
        self.fKink = 0
        self.hbowKink = 0
        self.hsternKink = 0
        self.fTime = time.time() 
        self.hbowTime = time.time()
        self.hsternTime = time.time()
        self.bearing = 0
        self.gotBearing = False
        t = threading.Thread(target=self.integrate)
        t.daemon = True
        t.start()

    def getDisplacement(self):
        self.update.acquire()
        r = (self.displacementE, self.displacementN, self.displacementZ)
        self.update.release()
        return r

    def resetDisplacement(self):
        self.update.acquire()
        self.displacementE = 0
        self.displacementN = 0
        self.displacementZ = 0
        self.update.release()

    def integrate(self):
        oldTime = time.time()
        while True:
            time.sleep(0.25)
            self.update.acquire()
            #self.update.wait()
            #time.sleep(1)
            if self.gotBearing == False:
                self.update.release()
                continue
            sb = math.sin(self.bearing)
            cb = math.cos(self.bearing)
            forwardSpeed = self.propSpeed() * settings.propSpeedScale / (time.time() - oldTime)
            strafeSpeed = ((self.hbowSpeed() + self.hsternSpeed()) / 2) * settings.strafeSpeedScale / (time.time() - oldTime)
            self.displacementE += sb * forwardSpeed
            self.displacementN += cb * forwardSpeed
            self.displacementE += cb * strafeSpeed
            self.displacementN += sb * strafeSpeed
            oldTime = time.time()
            self.update.release()
            info("Disp.: %gE %gN %gZ" % (self.displacementN, self.displacementE, self.displacementZ))
            info("Speed: %g (%g, %g)" % (self.propSpeed(), self.hbowSpeed(), self.hsternSpeed()))
            #print(self.fKink)
            #print(time.time() - self.fTime)
            #print((self.propSpeedReq - self.fKink) * (1 - math.exp(-settings.propSpeedCurve * (time.time() - self.fTime))))

    def propSpeed(self):
        return (self.propSpeedReq - self.fKink) * (1 - math.exp(-settings.propSpeedCurve * (time.time() - self.fTime))) + self.fKink

    def hbowSpeed(self):
        return (self.hbowSpeedReq - self.hbowKink) * (1 - math.exp(-settings.strafeSpeedCurve * (time.time() - self.hbowTime))) + self.hbowKink

    def hsternSpeed(self):
        return (self.hsternSpeedReq - self.hsternKink) * (1 - math.exp(-settings.strafeSpeedCurve * (time.time() - self.hsternTime))) + self.hsternKink
    
    def onMotorStateMessage(self, m):
        #onMotorMessage(self, m):
        if m.motorId == messaging.MotorID.Prop:
            #print 'FSpeed: %d' % (m.speed * propSpeedConstant)
            #print 'FSpeedReq: %d' % (m.speed * settings.propSpeedScale)
            self.update.acquire()
            self.fKink = self.propSpeed()
            self.propSpeedReq = m.speed
            self.fTime = time.time()
            #self.update.notify()
            self.update.release()
        elif m.motorId == messaging.MotorID.HBow:
            #print 'RSpeed from bow: %d' % (m.speed * strafeSpeedConstant)
            #print 'RBSpeedReq: %d' % (m.speed * settings.strafeSpeedScale)
            self.update.acquire()
            self.hbowKink = self.hbowSpeed()
            self.hbowSpeedReq = m.speed
            self.hbowTime = time.time()
            #self.update.notify()
            self.update.release()
        elif m.motorId == messaging.MotorID.HStern:
            #print 'RSpeed from stern: %d' % (m.speed * strafeSpeedConstant)
            #print 'RSSpeedReq: %d' % (m.speed * settings.strafeSpeedScale)
            self.update.acquire()
            self.hsternKink = self.hsternSpeed()
            self.hsternSpeedReq = m.speed
            self.hsternTime = time.time()
            #self.update.notify()
            self.update.release()

    def onTelemetryMessage(self, m):
        #onBearingAutopilotEnabledMessage(self, m):
        #if m.enabled == True:
            #print 'Bearing: %d' % m.orientation.yaw#target
            self.update.acquire()
            self.bearing = m.orientation.yaw#target
            self.displacementZ = -m.depth            
            #self.gotBearing == True
            self.gotBearing = True
            #self.update.notify()
            self.update.release()

