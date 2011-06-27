import cauv.messaging as messaging
from cauv.debug import debug, error, warning, info

import time
import threading
import math

# forward/reverse speed scale and exponential factors
class settings:
    fSpeedScale = 0.001
    rSpeedScale = 0.001
    fSpeedCurve = 0.5
    rSpeedCurve = 0.5
    

class DisplacementIntegrator:
    def __init__(self):
        self.displacementE = 0
        self.displacementN = 0
        self.displacementZ = 0
        self.timeLast = time.time()
        self.update = threading.Condition()
        #self.fSpeed = 0
        #self.rSpeedB = 0
        #self.rSpeedS = 0
        self.fSpeedReq = 0
        self.rBSpeedReq = 0
        self.rSSpeedReq = 0
        self.fKink = 0
        self.rBKink = 0
        self.rSKink = 0
        self.fTime = time.time() 
        self.rBTime = time.time()
        self.rSTime = time.time()
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

    def integrate(self):
        oldTime = time.time()
        while True:
            time.sleep(1)
            self.update.acquire()
            #self.update.wait()
            #time.sleep(1)
            if self.gotBearing == False:
                self.update.release()
                continue
            self.displacementE += math.sin(self.bearing) * self.fSpeed() * settings.fSpeedScale / (time.time() - oldTime)
            self.displacementN += math.cos(self.bearing) * self.fSpeed() * settings.fSpeedScale / (time.time() - oldTime)
            self.displacementE += math.cos(self.bearing) * (self.rBSpeed() + self.rSSpeed()) * settings.rSpeedScale / (time.time() - oldTime)
            self.displacementN += math.sin(self.bearing) * (self.rBSpeed() + self.rSSpeed()) * settings.rSpeedScale / (time.time() - oldTime)
            oldTime = time.time()
            self.update.release()
            info("Disp.: %gE %gN %gZ" % (self.displacementN, self.displacementE, self.displacementZ))
            info("Speed: %g (%g, %g)" % (self.fSpeed(), self.rBSpeed(), self.rSSpeed()))
            #print(self.fKink)
            #print(time.time() - self.fTime)
            #print((self.fSpeedReq - self.fKink) * (1 - math.exp(-settings.fSpeedCurve * (time.time() - self.fTime))))

    def fSpeed(self):
        return (self.fSpeedReq - self.fKink) * (1 - math.exp(-settings.fSpeedCurve * (time.time() - self.fTime))) + self.fKink

    def rBSpeed(self):
        return (self.rBSpeedReq - self.rBKink) * (1 - math.exp(-settings.rSpeedCurve * (time.time() - self.rBTime))) + self.rBKink

    def rSSpeed(self):
        return (self.rSSpeedReq - self.rSKink) * (1 - math.exp(-settings.rSpeedCurve * (time.time() - self.rSTime))) + self.rSKink
    
    def onMotorStateMessage(self, m):
        #onMotorMessage(self, m):
        if m.motorId == messaging.MotorID.Prop:
            #print 'FSpeed: %d' % (m.speed * fSpeedConstant)
            #print 'FSpeedReq: %d' % (m.speed * settings.fSpeedScale)
            self.update.acquire()
            #self.fSpeed = m.speed * fSpeedConstant
            #self.fSpeedReq = m.speed
            self.fKink = self.fSpeed()
            self.fSpeedReq = m.speed
            self.fTime = time.time()
            #self.update.notify()
            self.update.release()
        elif m.motorId == messaging.MotorID.HBow:
            #print 'RSpeed from bow: %d' % (m.speed * rSpeedConstant)
            #print 'RBSpeedReq: %d' % (m.speed * settings.rSpeedScale)
            self.update.acquire()
            #self.rSpeedB = m.speed * rSpeedConstant
            #self.rBSpeedReq = m.speed
            self.rBKink = self.rBSpeed()
            self.rBSpeedReq = m.speed
            self.rBTime = time.time()
            #self.update.notify()
            self.update.release()
        elif m.motorId == messaging.MotorID.HStern:
            #print 'RSpeed from stern: %d' % (m.speed * rSpeedConstant)
            #print 'RSSpeedReq: %d' % (m.speed * settings.rSpeedScale)
            self.update.acquire()
            #self.rSpeedS = m.speed * rSpeedConstant
            #self.rSSpeedReq = m.speed
            self.rSKink = self.rSSpeed()
            self.rSSpeedReq = m.speed
            self.rSTime = time.time()
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

