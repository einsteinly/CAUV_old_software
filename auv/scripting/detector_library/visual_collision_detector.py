#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import math

from cauv.debug import debug, info, warning, error
import cauv.pipeline as pipeline
from utils.movingaverage import MovingAverage

import AI

class VisualCollisionDetector(AI.Detector):
    class DefaultOptions(AI.Detector.DefaultOptions):
        def __init__(self):
            self.Channel = 'Value'
            self.No_Trigger = 3

    def __init__(self, node, opts):
        aiDetector.__init__(self, node, opts)
        self.request_pl('histogram.pipe')

        self.channel = self.options.Channel
        self.no_trigger = self.options.No_Trigger

        # A class to calculate the moving average of last maxcount number of
        # sample, and set trigger flag when sample is outside tolerance range:
        self.skewMovingMean = MovingAverage('lower', tolerance = 1, maxcount=30, st_multiplier=3)
        self.meanMovingMean = MovingAverage('upper', tolerance = 5, maxcount=30, st_multiplier=3)

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
                self.detected = True
                debug("Things just got brighter judging from changing skewness")
            elif self.meanMovingMean.trigger > self.no_trigger:
                self.detected = True
                debug("Things just got brighter judging from changing mean")
            else:
                self.detected = False

            debug('count %d' % self.skewMovingMean.count)
            debug('Moving average of skewness: %g' % self.skewMovingMean.movingMean)
            debug('Moving average of mean: %g' % self.meanMovingMean.movingMean)
            debug('Standard error of skewness: %g' % self.skewMovingMean.movingError)
            debug('Standard error of mean: %g' % self.meanMovingMean.movingError)

Detector = VisualCollisionDetector

if __name__ == "__main__":
    VisualCollisionDetector.entry()