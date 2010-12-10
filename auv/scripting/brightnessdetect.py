import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node

import time

class BrightnessDetect(messaging.BufferedMessageObserver):
    
   
    def __init__(self, node, channel='Value', skewTolerance=1, meanTolerance=5, maxcount=500):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("processing")
        node.addObserver(self)
        self.bin = bin
        self.skewTolerance = skewTolerance
        self.meanTolerance = meanTolerance
        self.maxcount = maxcount
        self.channel = channel        
        self.skewMovingMean=0
        self.meanMovingMean=0        
        self.count = 0
        self.detect = 0



    def onHistogramMessage(self, m):
        if m.type == self.channel:
 
            #Routine to display all the bin values
            accum = 0
            for i, bin in enumerate(m.bins):
                accum += bin
                print 'bin %d: %f, accum: %f' %(i, bin, accum)
 
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

            if self.count==0:
                self.skewMovingMean = skewness
                self.meanMovingMean = mean
                self.count+=1
            else:
                if skewness<self.skewMovingMean-self.skewTolerance:
                    self.detect = 1
                    print "Things just got brighter judging from changing skewness"
                else:
                    self.detect = 0
                    self.skewMovingMean = (self.count*self.skewMovingMean+skewness)/(self.count+1)
                        
                if mean>self.meanMovingMean+self.meanTolerance:
                    self.detect = 1
                    print "Things just got brighter judging from changing mean"
                else:
                    self.detect = 0
                    self.meanMovingMean = (self.count*self.meanMovingMean+mean)/(self.count+1)
                        
                if self.count < self.maxcount-2:
                    self.count += 1
 
            print 'count', self.count
            print 'Moving average of skewness: %f' %self.skewMovingMean
            print 'Moving average of mean: %f' %self.meanMovingMean

          


if __name__ == '__main__':
    node = cauv.node.Node('BrigDet')
    auv = control.AUV(node)
    detect = BrightnessDetect(node)
    while True:
        time.sleep(5)
