import cauv
import cauv.messaging as messaging
import cauv.control as control
import cauv.node
import threading
from math import degrees, cos, sin
import time

class PipeConfirmer(messaging.BufferedMessageObserver):
    def __init__(self, node, auv, centre_name, histogram_name, bin, strafe_p = 30, lower_threshold=0.05):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        self.auv = auv
        node.join("processing")
        node.addObserver(self)
        self.strafe_p = strafe_p
        self.centre_name = centre_name
        self.histogram_name = histogram_name
        self.binsStart = None
        self.binsPrevious = None
        self.ms = 1
        self.cv = threading.Condition()
        self.failed = False
        self.parallelsAppeared = False
        self.noLines = 0
        self.enabled = False

    def onCentreMessage(self, m):
        if m.name == self.centre_name and self.enabled == True:
            print 'Set strafe (confirm): %i' % (int((m.x - 0.5) * self.strafe_p))
            self.auv.strafe(int((m.x - 0.5) * self.strafe_p))

    def onHistogramMessage(self, m):
        if m.type == self.histogram_name and self.enabled == True:
            if self.binsStart == None:
                for j, indBin in enumerate(self.bin):
                    self.binsPrevious[j] = m.bins[indBin]
                    self.binsStart[j] = m.bins[indBin]
            if self.ms % 15 == 0:
                for j, indBin in enumerate(self.bin):
                   binsNow[j] = m.bins[indBin]
                if sum([x for x in self.binsNow]) < sum([x for x in self.binsPrevious]) - lower_threshold:
                    self.cv.acquire()
                    self.cv.notify()
                    self.failed = True
                    self.cv.release()
                #if sum([x for x in self.binsNow]) < sum([x for x in self.binsPrevious]) - 0.05:
                    #self.score -= 1
                #elif sum([x for x in self.binsNow]) > sum([x for x in self.binsPrevious]) + 0.05:
                    #self.score += 1
                #if score < -10 + frame * 
                    #self.cv.acquire()
                    #self.cv.notify()
                    #self.failed = True
                    #self.cv.release()
                self.binsPrevious = binsNow
            self.ms += 1

    def onHoughLinesMessage(self, m):
        if self.enabled == True:
            linesFound == False
            for i, line1 in enumerate(m.lines):
                for j, line2 in enumerate(m.lines):
                    if degrees(abs(line1.angle - line2.angle)) < 15 and i != j:
                        linesFound = True
            if parallelsAppeared == False and linesFound == True:
                parallelsAppeared = True
            if linesFound == False:   
                self.noLines += 1
            if self.noLines == 15:
                self.cv.acquire()
                self.cv.notify()
                self.failed = True
                self.cv.release()            

    def confirm(self):
        self.enabled = True
        self.cv.acquire()
        self.cv.wait(12)
        if self.failed == True:
            self.cv.release()
            return False

        if sum([x for x in self.binsNow]) < sum([x for x in self.binsStart] + 0.2):
            self.cv.release()
            return False

        self.cv.release()
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

    pf = PipeConfirmer(auv_node, auv, 'pipe', 'Hue', [11, 12])
    print pf.confirm()
    return pf
    
if __name__ == "__main__":
    setup()
    while True:
        time.sleep(5)
