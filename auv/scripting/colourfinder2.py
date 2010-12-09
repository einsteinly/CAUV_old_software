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
        self.maxcount = maxcount
        self.channel = channel        
        self.detect = 0
        self.binMovingmean = MovingAverage(upper, tolerance, maxcount)         #A class to calculate the moving average of last maxcount number of sample, and set trigger flag when sample is outside tolerance range

    def onHistogramMessage(self, m):
        if m.type == self.channel:
            self.binMovingmean.update(m.bins[self.bin])
            
            if self.binMovingmean.trigger == 1:
                self.detect = 1
                print "Bin %d is big" % self.bin
            else:
                self.detect = 0

            print 'Moving average of bin %d is: %f' %(self.bin, self.binMovingmean.movingMean)
            print 'Standard error of bin %d is: %f' %(self.bin, self.binMovingmean.movingError)
            
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
