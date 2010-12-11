import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node
from movingaverage import MovingAverage

import time

class BrightnessDetect(messaging.BufferedMessageObserver):
    
   
    def __init__(self, node, channel='Value', no_trigger = 3):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("processing")
        node.addObserver(self)
        self.channel = channel
        self.no_trigger = no_trigger        
        self.detect = 0
        self.skewMovingMean = MovingAverage('lower', tolerance = 1, maxcount=200, st_multiplier=2)         #A class to calculate the moving average of last maxcount number of sample, and set trigger flag when sample is outside tolerance range
        self.meanMovingMean = MovingAverage('upper', tolerance = 5, maxcount=200, st_multiplier=2)
    
    def print_bins(self, m):
        #Routine to display all the bin values
        accum = 0
        for i, bin in enumerate(m.bins):
            accum += bin
            print 'bin %d: %f, accum: %f' %(i, bin, accum)    

    def onHistogramMessage(self, m):
        if m.type == self.channel:
 
            #self.print_bins(m)
            
            #Routine to calculate the skewness of the histogram
            total1 = 0
            total2 = 0
            total3 = 0
            
            for i, bin in enumerate(m.bins):
                total1 += bin*i
                total2 += bin*i**2
                total3 += bin*i**3
            
            mean = total1
            stdev= (total2-mean**2)**0.5
            skewness = (total3-3*mean*stdev**2-mean**3)/stdev**3
            print 'Mean: %f, Skewness: %f' %(mean, skewness)

            #updating the statistics
            self.skewMovingMean.update(skewness) 
            self.meanMovingMean.update(mean)

            if self.skewMovingMean.trigger > self.no_trigger:
                self.detect = 1
                print "Things just got brighter judging from changing skewness"
                return 0
            elif self.meanMovingMean.trigger > self.no_trigger:
                self.detect = 1
                print "Things just got brighter judging from changing mean"
                return 0
            else:
                self.detect = 0
                        
            print 'count', self.skewMovingMean.count
            print 'Moving average of skewness:', self.skewMovingMean.movingMean
            print 'Moving average of mean:', self.meanMovingMean.movingMean
            print 'Standard error of skewness:', self.skewMovingMean.movingError
            print 'Standard error of mean:', self.meanMovingMean.movingError
          


if __name__ == '__main__':
    node = cauv.node.Node('BrigDet')
    auv = control.AUV(node)
    detect = BrightnessDetect(node)
    while True:
        time.sleep(5)
