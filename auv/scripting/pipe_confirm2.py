import cauv
import cauv.messaging as messaging
import cauv.control as control
import cauv.node
import threading
from math import degrees, cos, sin
import time

class PipeConfirmer(messaging.BufferedMessageObserver):
    def __init__(self, node, auv, bin, centre_name='pipe', histogram_name='Hue', strafe_p = 30, lower_threshold=0.05):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        self.auv = auv
        node.join("processing")
        node.addObserver(self)
        self.strafe_p = strafe_p
        self.centre_name = centre_name
        self.histogram_name = histogram_name
        self.bin = bin
        self.lower_threshold = lower_threshold
        self.binsStart = []
        self.binsPrevious = []
        self.binsNow = []
        self.ms = 1
        self.cv = threading.Condition()
        self.failed = False
        self.parallelsAppeared = False
        self.noLines = 0
        self.enabled = False
        self.foundparalle=0

    def onCentreMessage(self, m):
        if m.name == self.centre_name and self.enabled == True:
            print 'Set strafe (confirm): %i' % (int((m.x - 0.5) * self.strafe_p))
            self.auv.strafe(int((m.x - 0.5) * self.strafe_p))

    def onHistogramMessage(self, m):
        if m.type == self.histogram_name and self.enabled == True:
            if self.binsStart == []:
                for indBin in self.bin:
                    self.binsPrevious.append(m.bins[indBin])
                    self.binsStart.append(m.bins[indBin])
            if self.ms % 15 == 0:
                for indBin in self.bin:
                   self.binsNow.append(m.bins[indBin])
                if sum(self.binsNow) < sum(self.binsPrevious) - self.lower_threshold:
                    self.cv.acquire()
                    self.cv.notify()
                    self.failed = True
                    self.cv.release()
                self.binsPrevious = self.binsNow
            self.ms += 1

    def onHoughLinesMessage(self, m):
        if self.enabled == True:
            self.linesFound = False
            for i, line1 in enumerate(m.lines):
                for j, line2 in enumerate(m.lines):
                    if degrees(abs(line1.angle - line2.angle)) < 15 and i != j:
                        self.foundparalle += 1
     

    def confirm(self):
        print 'hi'
        self.enabled = True
        self.cv.acquire()
        self.cv.wait(12)
        self.enabled = False
        
        if self.failed == True:
            self.cv.release()
            print 'Not confirmed'
            return False

        if self.foundparalle < 10:
            print 'Not confirmed'
            return False

        if sum(self.binsNow) < sum(self.binsStart):
            self.cv.release()
            print 'Not confirmed'
            return False
            
        self.cv.release()
        print 'Confirmed'
        return True

        
def setup():
    auv_node = cauv.node.Node('py-auv-pc')                #Create a node of the spread messaging service
    auv = control.AUV(auv_node)                        #Create a python object for the control of the AUV
    
    print 'setting calibration...'                #setting the y intercept and gradient of the pressure/depth curve for front and back pressure sensor
    # set-up calibration factors
    auv_node.send(messaging.DepthCalibrationMessage(
        -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    ), "control")

    auv.bearingParams(1, 0, -80, 1)                #Setting kp kd ki and scale of the following parameters
    auv.depthParams(40, 0, 0, 1)
    #auv.pitchParams(1, 0, 0, 1)

    auv.propMap(10, -10, 127, -127)
    auv.vbowMap(10, -10, 127, -127)
    auv.hbowMap(10, -10, 127, -127)
    auv.vsternMap(10, -10, 127, -127)
    auv.hsternMap(10, -10, 127, -127)

    pf = PipeConfirmer(auv_node, auv, [11, 12])
    print pf.confirm()
    return pf
    
if __name__ == "__main__":
    setup()
    while True:
        time.sleep(5)
