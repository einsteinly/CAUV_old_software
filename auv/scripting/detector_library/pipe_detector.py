from AI_classes import aiDetector
from cauv.debug import debug, info, warning, error

import cauv.node
import cauv.messaging as messaging
import time
import threading

class detectorOptions:
    Bin_Number = 8
    Bin_Threshold = 0.05
    Required_Pipeline = 'detect_pipe.pipe'
    Histogram_Name = 'pipe-detect'

class detector(aiDetector):
    def __init__(self, node):
        aiDetector.__init__(self, node)
        self.node.join("processing")
        
        info("Pipe detector loaded")
        if detectorOptions.Required_Pipeline:
            try:
                self.request_pl(detectorOptions.Required_Pipeline)
            except Exception, e:
                warning('Pipe Detector pipeline request failed: %s' % e)

        self.detected = False
        
    def process(self):
        if self.detected:
            debug('yellow object visible')
        else:
            pass

    def onHistogramMessage(self, m):
        if m.name != PipeDetectorOptions.Histogram_Name:
            return
        if m.bins[detectorOptions.Bin_Number] > detectorOptions.Bin_Threshold:
            self.detected = True
        else:
            self.detected = False

if __name__ == '__main__':
    dt = detector()
    while True:
        dt.process()
        time.sleep(1)
