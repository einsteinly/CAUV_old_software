from AI_classes import aiDetector, aiDetectorOptions
from cauv.debug import debug, info, warning, error

import cauv.node
import cauv.messaging as messaging
from utils.timeaverage import TimeAverage
import time
import threading
import math

class detectorOptions(aiDetectorOptions):
    Threshold = 0.38
    Required_Pipeline = 'detect_pipe'
    Float_Name = 'pipe_det'
    Lines_Name = 'pipe_det'
    Angular_Discrepancy = 0.5
    useLines = True

class detector(aiDetector):
    debug_values = ['detected', 'colour_trigger']
    def __init__(self, node, opts):
        aiDetector.__init__(self, node, opts)
        self.start_time = time.time()

        try:
            self.request_pl(self.options.Required_Pipeline)
        except Exception, e:
            warning('Pipe Detector pipeline request failed: %s' % e)

        self.colour_trigger = 0
        self.detected = False
        self.node.join("processing")

    def process(self):
        if self.detected:
            debug('yellow object visible')
        else:
            pass

    def onFloatMessage(self, m):
        print m
        if m.name != self.options.Float_Name:
            return
        self.colour_trigger = m.value
        if not self.options.useLines:
            if self.colour_trigger > self.options.Threshold:
                self.detected = True
            else:
                self.detected = False
        
    def onLinesMessage(self, m):
        if not self.options.useLines:
            return
        if m.name != self.options.Lines_Name:
            return
        if not len(m.lines):
            self.detected = False
            return
        initial_angle = m.lines[0].angle
        for line in m.lines:
            if abs(initial_angle-line.angle)%math.pi<self.options.Angular_Discrepancy or abs(initial_angle-line.angle)%math.pi>math.pi-self.options.Angular_Discrepancy:
                self.detected = False
                return
        if self.colour_trigger > self.options.Threshold:
            self.detected = True
            debug ("Detected pipe!")
        else:
            self.detected = False

if __name__ == '__main__':
    dt = detector()
    while True:
        dt.process()
        time.sleep(1)