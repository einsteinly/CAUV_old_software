import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node

import time
import threading
import math

class Displacement(messaging.MessageObserver):

    def __init__(self, node):
        messaging.MessageObserver.__init__(self)
        self.__node = node
        #node.join("control")
	node.join("telemetry")	
	node.join("gui")
        node.addObserver(self)
        self.displacementE = 0
        self.displacementN = 0
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
	self.fSpeedScale = 1
	self.rSpeedScale = 1
	self.fSpeedCurve = 0.5
	self.rSpeedCurve = 0.5

    def getDisplacement(self):
        self.update.acquire()
        r = math.sqrt(pow(displacementE, 2) + pow(displacementN, 2))
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
	    self.displacementE += math.sin(self.bearing) * self.fSpeed() * self.fSpeedScale / (time.time() - oldTime)
            self.displacementN += math.cos(self.bearing) * self.fSpeed() * self.fSpeedScale / (time.time() - oldTime)
            self.displacementE += math.cos(self.bearing) * (self.rBSpeed() + self.rSSpeed()) * self.rSpeedScale / (time.time() - oldTime)
	    self.displacementN += math.sin(self.bearing) * (self.rBSpeed() + self.rSSpeed()) * self.rSpeedScale / (time.time() - oldTime)
            oldTime = time.time()
	    self.update.release()
	    print(("Disp.: %d " % self.displacementN) + ("%d" % self.displacementE))
            print(self.fSpeed())
            #print(self.fKink)
            #print(time.time() - self.fTime)
            #print((self.fSpeedReq - self.fKink) * (1 - math.exp(-self.fSpeedCurve * (time.time() - self.fTime))))
	    print(self.rBSpeed())
	    print(self.rSSpeed())

    def fSpeed(self):
        return (self.fSpeedReq - self.fKink) * (1 - math.exp(-self.fSpeedCurve * (time.time() - self.fTime))) + self.fKink

    def rBSpeed(self):
        return (self.rBSpeedReq - self.rBKink) * (1 - math.exp(-self.rSpeedCurve * (time.time() - self.rBTime))) + self.rBKink

    def rSSpeed(self):
        return (self.rSSpeedReq - self.rSKink) * (1 - math.exp(-self.rSpeedCurve * (time.time() - self.rSTime))) + self.rSKink
    
    def onMotorStateMessage(self, m):
        #onMotorMessage(self, m):
        print("Motor")
	if m.motorId == messaging.MotorID.Prop:
            #print 'FSpeed: %d' % (m.speed * fSpeedConstant)
            print 'FSpeedReq: %d' % (m.speed * self.fSpeedScale)
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
	    print 'RBSpeedReq: %d' % (m.speed * self.rSpeedScale)
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
	    print 'RSSpeedReq: %d' % (m.speed * self.rSpeedScale)
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
            #self.gotBearing == True
            self.gotBearing = True
            #self.update.notify()
            self.update.release()

if __name__ == '__main__':
    node = cauv.node.Node('Disp')
    auv = control.AUV(node)
    d = Displacement(node)
    d.integrate()
    while True:
        time.sleep(5)
