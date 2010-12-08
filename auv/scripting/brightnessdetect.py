import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node

import time

class BrightnessDetect(messaging.BufferedMessageObserver):
    
   
    def __init__(self, node, bin, channel='Value', tolerance=0.1, maxcount=500):
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
            
            mean = total1 / len(m.bins)
            stdev= (total2/len(m.bins)-mean**2)**0.5
            skewness = (total3/len(m.bins)-3*mean*stdev**2-mean**3)/stdev**3
            print 'Skewness:', skewness
            
            if skewness<self.movingMean-self.tolerance:
                self.detect = 1
                print "Things just got brighter"
                
            else:
                self.detect = 0
                if self.count==0:
                    self.movingMean = skewness
                else:
                    self.movingMean = (self.count*self.movingMean+skewness)/(self.count+1)
                    
                if self.count < self.maxcount-2:
                    self.count += 1
                print 'Moving average of skewness: %f' %self.movingMean
          


if __name__ == '__main__':
    node = cauv.node.Node('BrigDet')
    auv = control.AUV(node)
    detect = BrightnessDetect(node, 14)
    while True:
        time.sleep(5)
