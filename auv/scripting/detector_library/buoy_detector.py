from AI_classes import aiDetector
from cauv.debug import debug, info, warning, error

import cauv.pipeline as pipeline

import time
import sys
import math

from utils import vecops
from utils.hacks import incFloat 
from utils.detectors import ColourDetector

class BuoyDetectorOptions:
    Sightings_Period   = 3.0 # seconds, period to consider sightings of the buoy for
    Required_Confidence = 0.7
    Required_Pipeline = 'detect_buoy.pipe'
    Circles_Name = 'buoy'
    Histogram_Name = 'buoy'
    Histogram_Bin = 10 #17
    Stddev_Mult = 0.2 # lower --> higher confidence when multiple circles with different centres are visible
    Optimal_Colour_Frac = 0.04 # highest colour detection confidence when the specified bin contains this value
    Colour_Weight  = 0.3 # respective weightings in confidence, arithmetic mean of these values should be 1
    Circles_Weight = 1.7 #

class detector(aiDetector):
    def __init__(self, node):
        aiDetector.__init__(self, node)
        self.circles_messages = {}   # map time received : message
        self.colour_detector = ColourDetector(
            BuoyDetectorOptions.Sightings_Period,
            BuoyDetectorOptions.Histogram_Bin,
            BuoyDetectorOptions.Optimal_Colour_Frac
        )
        self.tzero = time.time()
        self.node.join('processing')
        self.node.join('pl_gui')
        if BuoyDetectorOptions.Required_Pipeline:
            try:
                self.request_pl(BuoyDetectorOptions.Required_Pipeline)
            except Exception, e:
                warning('Buoy Detector pipeline request failed: %s' % e)
    
    def relativeTime(self):
        return time.time() - self.tzero

    def process(self):
        # process recorded observations of circles, and set self.detected True
        # / False as necessary
        tnow = self.relativeTime()
        self.cullOldSightings(tnow - BuoyDetectorOptions.Sightings_Period)
        sightings = []
        numcircles = len(self.circles_messages.items())
        for t, m in  self.circles_messages.items():
            sightings.append(self.detectionConfidence(m))
        if numcircles:
            circles_confidence = BuoyDetectorOptions.Circles_Weight * sum(sightings[:numcircles]) / numcircles
        else:
            circles_confidence = 0
        colour_confidence = BuoyDetectorOptions.Colour_Weight * self.colour_detector.confidence()
        confidence = circles_confidence + colour_confidence
        info('buoy detector processing %d sightings, confidence=%g (circles=%g,colour=%g)' % (
            len(sightings), confidence, circles_confidence, colour_confidence))
        if confidence > BuoyDetectorOptions.Required_Confidence:
            info('buoy detected, confidence = %g, %d sightings' % (confidence, len(sightings)))
            self.detected = True
        else:
            self.detected = False
    
    def die(self):
        try:
            self.drop_all_pl()
        except Exception, e:
            warning('Buoy Detector pipeline drop request failed: %s' % e)

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
            self.colour_detector.update(m)
        else:
            debug('ignoring histogram message: %s' % m.name, 2)
