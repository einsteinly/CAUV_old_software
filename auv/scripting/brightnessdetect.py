import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node
from cauv.debug import debug, warning, error, info
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
        # A class to calculate the moving average of last maxcount number of
        # sample, and set trigger flag when sample is outside tolerance range:
        self.skewMovingMean = MovingAverage('lower', tolerance = 1, maxcount=30, st_multiplier=3)
        self.meanMovingMean = MovingAverage('upper', tolerance = 5, maxcount=30, st_multiplier=3)

    def print_bins(self, m):
        #Routine to display all the bin values
        accum = 0
        message = ''
        for i, bin in enumerate(m.bins):
            accum += bin
            message += 'bin %d: %f, accum: %f\n' % (i, bin, accum)
        debug(message)

    def onHistogramMessage(self, m):
        if m.name == self.channel:
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
            debug('Mean: %f, Skewness: %f' %(mean, skewness))

            #updating the statistics
            self.skewMovingMean.update(skewness)
            self.meanMovingMean.update(mean)

            if self.skewMovingMean.trigger > self.no_trigger:
                self.detect = 1
                debug("Things just got brighter judging from changing skewness")
                self.emergencyStop()
                return 0
            elif self.meanMovingMean.trigger > self.no_trigger:
                self.detect = 1
                debug("Things just got brighter judging from changing mean")
                self.emergencyStop()
                return 0
            else:
                self.detect = 0

            debug('count %d' % self.skewMovingMean.count)
            debug('Moving average of skewness: %g' % self.skewMovingMean.movingMean)
            debug('Moving average of mean: %g' % self.meanMovingMean.movingMean)
            debug('Standard error of skewness: %g' % self.skewMovingMean.movingError)
            debug('Standard error of mean: %g' % self.meanMovingMean.movingError)


    def emergencyStop(self):
            auv.prop(-127)
            warning('Emergency stop')
            time.sleep(2)
            auv.prop(0)


if __name__ == '__main__':
    node = cauv.node.Node('BrigDet')
    auv = control.AUV(node)
    auv.prop(100)
    debug('Moving forward')
    detect = BrightnessDetect(node)
    while True:
        time.sleep(1.0)
