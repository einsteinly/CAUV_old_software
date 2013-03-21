import time
import threading
import math

import cauv.node
import cauv.messaging as messaging
from cauv.debug import debug, info, warning, error
from utils.timeaverage import TimeAverage

import AI

class PipeDetector(AI.Detector):
    class detectorOptions(AI.Detector.DefaultOptions):
        def __init__(self):
            self.Threshold = 0.39
            self.Float_Name = 'pipe_det'
            self.Lines_Name = 'pipe_det'
            self.Angular_Discrepancy = 0.5
            self.useLines = True
            self.pipelines = 'detect_pipe'

    class Debug(AI.Detector.Debug):
        def __init__(self):
            self.detected = False
            self.colour_trigger = 0
            
    def __init__(self, node, opts):
        aiDetector.__init__(self, node, opts)
        self.start_time = time.time()

        self.colour_trigger = 0.0
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
            debug(str(abs(initial_angle-line.angle)%math.pi))
            if abs(initial_angle-line.angle)%math.pi>self.options.Angular_Discrepancy and abs(initial_angle-line.angle)%math.pi<math.pi-self.options.Angular_Discrepancy:
                self.detected = False
                return
        if self.colour_trigger > self.options.Threshold:
            self.detected = True
            debug ("Detected pipe!")
        else:
            self.detected = False

Detector = PipeDetector

if __name__ == "__main__":
    PipeDetector.entry()
