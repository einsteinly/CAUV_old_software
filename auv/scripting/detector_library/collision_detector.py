from AI_classes import aiDetector
from cauv.debug import debug, info, warning, error

import cauv.pipeline as pipeline

import math
from movingaverage import MovingAverage

class CollisionAvoidanceOptions:
    Pipeline_File = 'pipelines/histogram.pipe'
    Load_Pipeline = None #'buoy-detect'
    Channel = 'Value'
    No_Trigger = 3

class detector(aiDetector):
    def __init__(self, node):
        aiDetector.__init__(self, node)
        self.__node = node
        self.node.join('processing')
        node.addObserver(self)

        self.channel = CollisionAvoidanceOptions.Channel
        self.no_trigger = CollisionAvoidanceOptions.No_Trigger

        # A class to calculate the moving average of last maxcount number of
        # sample, and set trigger flag when sample is outside tolerance range:
        self.skewMovingMean = MovingAverage('lower', tolerance = 1, maxcount=30, st_multiplier=3)
        self.meanMovingMean = MovingAverage('upper', tolerance = 5, maxcount=30, st_multiplier=3)

        self.__pl = pipeline.Model(self.node, CollisionAvoidanceOptions.Load_Pipeline)
        if CollisionAvoidanceOptions.Load_Pipeline is not None:        
            self.__pl.load(CollisionAvoidanceOptions.Pipeline_File)
    
    def die(self):
        # save the running pipeline in case someone edited it!
        if CollisionAvoidanceOptions.Load_Pipeline is not None:
            info('saving the current %s pipeline to %s' % (CollisionAvoidanceOptions.Load_Pipeline,
                                                           CollisionAvoidanceOptions.Pipeline_File))
            self.__pl.load(CollisionAvoidanceOptions.Pipeline_File + '.autosaved')

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

