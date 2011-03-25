import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node

import time
import threading
import math

class Displacement(messaging.BufferedMessageObserver):

    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("control")
        node.addObserver(self)
        self.displacementE = 0
        self.displacementN = 0
        self.timeLast = time.time()
        self.update = threading.Condition()
        self.speed = 0
        self.bearing = 0
        self.gotBearing = False
	self.speedConstant = 1

    def getDisplacement(self):
        self.update.acquire()
        r = math.sqrt(pow(displacementE, 2) + pow(displacementN, 2))
        self.update.release()
        return r

    def integrate(self):
        while True:
            print("test")
	    self.update.acquire()
            self.update.wait()
            if self.gotBearing == False:
                continue
            self.displacementE += math.sin(self.bearing) * self.speed
            self.displacementN += math.cos(self.bearing) * self.speed
            self.update.release()

    def onMotorStateMessage(self, m):
        #onMotorMessage(self, m):
        if m.motorId == messaging.MotorID.Prop:
            print 'Speed: %d' % (im.speed * speedConstant)
            self.update.acquire()
            self.speed = m.speed * speedConstant
            self.update.notify()
            self.update.release()

    def onTelemetryMessage(self, m):
        #onBearingAutopilotEnabledMessage(self, m):
        #if m.enabled == True:
            print 'Bearing: %d' % m.orentation#target
            self.update.acquire()
            self.bearing = m.orientation#target
            self.gotBearing == True
            self.update.notify()
            self.update.release()

if __name__ == '__main__':
    node = cauv.node.Node('Disp')
    auv = control.AUV(node)
    print "1"
    d = Displacement(node)
    print "2"
    d.integrate()
    print "3"
    while True:
        time.sleep(5)
