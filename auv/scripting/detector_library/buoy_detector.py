from AI_classes import aiDetector
from cauv.debug import debug, info, warning, error

import cauv.pipeline as pipeline

import time
import sys
import math

from utils import vecops

class BuoyDetectorOptions:
    Sightings_Period   = 3.0 # seconds, period to consider sightings of the buoy for
    Required_Confidence = 0.7
    Pipeline_File = 'pipelines/detect_buoy.pipe'
    Load_Pipeline = None #'buoy-detect'
    Circles_Name = 'buoy'
    Histogram_Name = 'buoy'
    Histogram_Bin = 17
    Stddev_Mult = 0.2 # lower --> higher confidence when multiple circles with different centres are visible
    Optimal_Colour_Frac = 0.04 # highest colour detection confidence when the specified bin contains this value
    Colour_Weight  = 0.3 # respective weightings in confidence, arithmetic mean of these values should be 1
    Circles_Weight = 1.7 #

def incFloat(f):
    if f == 0.0:
        return sys.float_info.min
    m, e = math.frexp(f)
    return math.ldexp(m + sys.float_info.epsilon / 2, e)


class detector(aiDetector):
    def __init__(self, node):
        aiDetector.__init__(self, node)
        self.circles_messages = {}   # map time received : message
        self.histogram_messages = {} # "
        self.tzero = time.time()
        self.node.join('processing')
        self.node.join('pl_gui')
        self.__pl = pipeline.Model(self.node, BuoyDetectorOptions.Load_Pipeline)
        if BuoyDetectorOptions.Load_Pipeline is not None:        
            self.__pl.load(BuoyDetectorOptions.Pipeline_File)
    
    def relativeTime(self):
        return time.time() - self.tzero

    def process(self):
        # process recorded observations of circles, and set self.detected True
        # / False as necessary
        tnow = self.relativeTime()
        self.cullOldSightings(tnow - BuoyDetectorOptions.Sightings_Period)
        sightings = []
        numcircles = len(self.circles_messages.items())
        numcolours = len(self.histogram_messages.items())
        for t, m in  self.circles_messages.items():
            sightings.append(BuoyDetectorOptions.Circles_Weight * self.detectionConfidence(m))
        for t, m in self.histogram_messages.items():
            sightings.append(BuoyDetectorOptions.Colour_Weight * self.colorDetectionConfidence(m))
        if numcircles:
            circles_confidence = sum(sightings[:numcircles]) / numcircles
        else:
            circles_confidence = 0
        if numcolours:
            colour_confidence = sum(sightings[numcolours:]) / numcolours
        else:
            colour_confidence = 0
        if len(sightings):
            confidence = sum(sightings) / len(sightings)
        else:
            confidence = 0
        info('buoy detector processing %d sightings, confidence=%g (circles=%g,colour=%g)' % (
            len(sightings), confidence, circles_confidence, colour_confidence))
        if confidence > BuoyDetectorOptions.Required_Confidence:
            info('buoy detected, confidence = %g, %d sightings' % (confidence, len(sightings)))
            self.detected = True
        else:
            self.detected = False
    
    def die(self):
        # save the running pipeline in case someone edited it!
        if BuoyDetectorOptions.Load_Pipeline is not None:
            info('saving the current %s pipeline to %s' % (BuoyDetectorOptions.Load_Pipeline,
                                                           BuoyDetectorOptions.Pipeline_File))
            self.__pl.load(BuoyDetectorOptions.Pipeline_File + '.autosaved')
    
    def colorDetectionConfidence(self, histogram_message):
        frac = histogram_message.bins[BuoyDetectorOptions.Histogram_Bin]
        confidence = min(
            1, frac*BuoyDetectorOptions.Optimal_Colour_Frac / abs(BuoyDetectorOptions.Optimal_Colour_Frac - frac)
        )
        debug('color confidence=%g, frac=%g' % (confidence, frac), 5)
        return confidence

    def detectionConfidence(self, message):
        if len(message.circles) == 0:
            return 0
        sx_radius = sum(map(lambda x: x.radius, message.circles))
        mean_radius = sx_radius / len(message.circles)
        sx  = vecops.sx(map(lambda x: x.centre, message.circles))
        sxx = vecops.sxx(map(lambda x: x.centre, message.circles))
        stddev = vecops.pow(vecops.absv(vecops.sub(sxx,vecops.sqr(sx))), 0.5)
        s = (stddev.x + stddev.y) / 2
        confidence = 1 / (1 + BuoyDetectorOptions.Stddev_Mult * s/mean_radius)
        if len(message.circles) == 1:
            confidence = 0.9
        debug('circle centres s=%g, confidence=%g mean radius=%g' % (s, confidence, mean_radius), 5)
        return confidence

    def cullOldSightings(self, cull_before_time):
        def cull(d):
            to_remove = []
            for t in d:
                if t < cull_before_time:
                    to_remove.append(t)
            for t in to_remove:
                del d[t]
        cull(self.circles_messages)
        cull(self.histogram_messages)

    def onCirclesMessage(self, m):
        if m.name == BuoyDetectorOptions.Circles_Name:
            # assuming time collisions are not going to happen very often!
            t = self.relativeTime()
            while t in self.circles_messages:
                # twiddle twiddle
                t = incFloat(t)
            self.circles_messages[t] = m
        else:
            debug('ignoring circles message: %s' % m.name, 2)
 
    def onHistogramMessage(self, m):
        if m.name == BuoyDetectorOptions.Histogram_Name:
            t = self.relativeTime()
            while t in self.histogram_messages:
                # twiddle twiddle
                t = incFloat(t)
            self.histogram_messages[t] = m
        else:
            debug('ignoring histogram message: %s' % m.name, 2)

