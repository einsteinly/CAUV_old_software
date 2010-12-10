import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node

from movingaverage import MovingAverage

import time

class ColourFinder(messaging.BufferedMessageObserver):

    def __init__(self, node, bin, channel = 'Hue', tolerance=0.1, maxcount=500):

        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("processing")
        node.addObserver(self)
        self.bin = bin
        self.tolerance = tolerance
        self.maxcount = maxcount
        self.channel = channel        
        self.movingMean=0
        self.count = 0
        self.detect = 0

    def onHistogramMessage(self, m):
        if m.type == self.channel:
            if m.bins[self.bin]>self.movingMean+self.tolerance:
                self.detect = 1
                print "Bin %d is big" % self.bin
            else:
                self.detect = 0
                if self.count==0:
                    self.movingMean = m.bins[self.bin]
                else:
                    self.movingMean = (self.count*self.movingMean+m.bins[self.bin])/(self.count+1)
                    
                if self.count < self.maxcount-2:
                    self.count += 1
                print 'Moving average of bin %d: %f' %(self.bin, self.movingMean)
          
            accum = 0
            for i, bin in enumerate(m.bins):
                accum += bin
                print 'bin %d: %f, accum: %f' %(i, bin, accum)

if __name__ == '__main__':
    node = cauv.node.Node('ColFind')
    auv = control.AUV(node)
    detect = ColourFinder(node, 14, 'Value')
    while True:
        time.sleep(5)
