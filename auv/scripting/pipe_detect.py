import cauv.node
import cauv.messaging as messaging
import time
import threading

class detector(messaging.BufferedMessageObserver):
    def __init__(self):
        self.node = cauv.node.Node('pydetpip')
        messaging.BufferedMessageObserver.__init__(self)
        self.node.join("processing")
        self.node.addObserver(self)
        self.lock = threading.Lock()
        self.detect = 0
    def process(self):
        if self.detected():
            print 'found yellow'
        else:
            print 'no yello'
    def onHistogramMessage(self, m):
        if m.bins[0]>0.1:
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
