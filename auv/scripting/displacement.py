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

    def getDisplacement(self):
        self.update.acquire()
        r = math.sqrt(pow(displacementE, 2) + pow(displacementN, 2))
        self.update.release()
        return r

    def integrate(self):
        while True:
            self.update.acquire()
            self.update.wait()
            displacementE += math.sin(bearing) * speed
            displacementN += math.cos(bearing) * speed
            self.update.release()

    def onMotorMessage(self, m):
        if motorId == messaging.MotorID.Prop:
            print 'Speed: %d' % m.speed
            self.update.acquire()
            self.speed = m.speed
            self.update.notify()
            self.update.release()

    def onBearingAutopilotEnabledMessage(self, m):
        if enabled == True:
            print 'Bearing: %d' % m.target
            self.update.acquire()
            self.bearing = m.target
            self.update.notify()
            self.update.release()

if __name__ == '__main__':
    node = cauv.node.Node('Displacement')
    auv = control.AUV(node)
    d = Displacement(node)
    d.integrate()
    while True:
        time.sleep(5)
