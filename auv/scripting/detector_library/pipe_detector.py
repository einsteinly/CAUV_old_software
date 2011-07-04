from AI_classes import aiDetector, aiDetectorOptions
from cauv.debug import debug, info, warning, error

import cauv.node
import cauv.messaging as messaging
from utils.timeaverage import TimeAverage
import time
import threading

class detectorOptions(aiDetectorOptions):
    Bin_Number = 8
    Bin_Threshold = 0.05
    Required_Pipeline = 'detect_pipe.pipe'
    Histogram_Name = 'pipe-detect'
    Average_Time = 2


class detector(aiDetector):
    def __init__(self, node, opts):
        aiDetector.__init__(self, node, opts)
        self.node.join("processing")

        self.intensity = TimeAverage(self.options.Average_Time)
        
        info("Pipe detector loaded")
        if self.options.Required_Pipeline:
            try:
                self.request_pl(self.options.Required_Pipeline)
            except Exception, e:
                warning('Pipe Detector pipeline request failed: %s' % e)

        self.detected = False
        
    def process(self):
        if self.detected:
            debug('yellow object visible')
        else:
            pass

    def onHistogramMessage(self, m):
        if m.name != self.options.Histogram_Name:
            return
        debug("Trigger at %f percent" % ((m.bins[self.options.Bin_Number] / self.options.Bin_Threshold) * 100))
        averageIntensity = self.intensity.update(m.bins[self.options.Bin_Number])
        if averageIntensity > self.options.Bin_Threshold:
            self.detected = True
            debug ("Detected pipe!")
        else:
            self.detected = False
            debug ("No pipe...")

if __name__ == '__main__':
    dt = detector()
    while True:
        dt.process()
        time.sleep(1)
