from AI_classes import aiDetector
from cauv.debug import debug, info, warning, error

import cauv.node
import cauv.messaging as messaging
import time
import threading

class Options:
    Bin_Number = 8
    Bin_Threshold = 0.05
    Pipeline_File = 'pipelines/detect_pipe.pipe'
    Load_Pipeline = 'pipe-detect'
    Histogram_Name = 'pipe-detect'

class detector(messaging.BufferedMessageObserver):
    def __init__(self, node):
        aiDetector.__init__(self, node)
        self.node.join("processing")
        self.lock = threading.Lock()
        self.detect = 0

    def process(self):
        if self.detected():
            debug('yellow object visible')
        else:
            pass

    def die():
        # if anything needs doing when detector is stopped, do it here
        pass

    def onHistogramMessage(self, m):
        if name != Options.Histogram_Name:
            return
        if m.bins[Options.Bin_NUmber] > Options.Bin_Threshold:
            self.lock.acquire()
            self.detect = 1
            self.lock.release()
        else:
            self.lock.acquire()
            self.detect = 0
            self.lock.release()
    def detected(self):
        self.lock.acquire()
        d = self.detect
        self.lock.release()
        return d
        
if __name__ == '__main__':
    dt = detector()
    while True:
        dt.process()
        time.sleep(1)
