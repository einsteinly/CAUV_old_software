from AI_classes import aiDetector, aiDetectorOptions
from cauv.debug import debug, info, warning, error

import cauv.pipeline as pipeline
from cauv.messaging import floatXYZ

import time
import sys
import math

from utils import vecops
from utils.hacks import incFloat 
from utils.detectors import ColourDetector

class detectorOptions(aiDetectorOptions):
    Sightings_Period   = 5.0 # seconds, period to consider sightings of the buoy for
    Required_Confidence = 0.9
    Required_Sightings = 5
    Required_Pipeline = 'detect_buoy_sim'
    Circles_Name = 'buoy'
    Histogram_Name_A = 'buoy_hue'
    Histogram_Name_B = 'buoy_hue'
    Histogram_Bins_A = xrange(60,120) # of 0--180 range
    Histogram_Bins_B = xrange(50,75) # of 0--180 range
    Stddev_Mult = 0.2 # lower --> higher confidence when multiple circles with different centres are visible
    Optimal_Colour_Frac_A = 0.003 # highest colour detection confidence when the specified bin contains this value
    Optimal_Colour_Frac_B = 0.008 # highest colour detection confidence when the specified bin contains this value
    Colour_Weight_A  = -20.0 # respective weightings in confidence
    Colour_Weight_B  =  0.4 #
    Circles_Weight   =  1.6 #

class detector(aiDetector):
    debug_values = []
    def __init__(self, node, opts):
        aiDetector.__init__(self, node, opts)
        self.circles_messages = {}   # map time received : message
        self.colour_detector_A = ColourDetector(
            self.options.Sightings_Period,
            bin_num = None,
            optimal_colour_frac = self.options.Optimal_Colour_Frac_A,
            bins = self.options.Histogram_Bins_A
        )
        self.colour_detector_B = ColourDetector(
            self.options.Sightings_Period,
            bin_num = None,
            optimal_colour_frac = self.options.Optimal_Colour_Frac_B,
            bins = self.options.Histogram_Bins_B
        )
        self.tzero = time.time()
        self.node.join('processing')
        self.node.join('pl_gui')
        if self.options.Required_Pipeline:
           try:
               self.request_pl(self.options.Required_Pipeline)
           except Exception, e:
               warning('Buoy Detector pipeline request failed: %s' % e)
        self.log('Looking for the buoy')
        self.process_c = 0
    
    def relativeTime(self):
        return time.time() - self.tzero

    def process(self):
        # process recorded observations of circles, and set self.detected True
        # / False as necessary
        self.process_c += 1
        tnow = self.relativeTime()
        self.cullOldSightings(tnow - self.options.Sightings_Period)
        sightings = []
        numcircles = len(self.circles_messages.items())
        for t, m in  self.circles_messages.items():
            sightings.append(self.detectionConfidence(m))
        if numcircles:
            circles_confidence = self.options.Circles_Weight * sum(sightings[:numcircles]) / numcircles
        else:
            circles_confidence = 0
        colour_confidence_A = self.options.Colour_Weight_A * self.colour_detector_A.confidence()
        colour_confidence_B = self.options.Colour_Weight_B * self.colour_detector_B.confidence()
        confidence = circles_confidence + colour_confidence_A + colour_confidence_B
        info('buoy detector processing %d sightings, confidence=%g (circles=%g,colour_A=%g,colour_B=%g)' % (
            len(sightings), confidence, circles_confidence, colour_confidence_A, colour_confidence_B
        ))
        debug('mean frac: A=%g B=%g' % (self.colour_detector_A.frac(), self.colour_detector_B.frac()))
        if confidence >= self.options.Required_Confidence and \
           len(sightings) >= self.options.Required_Sightings:
            info('buoy detected, confidence = %g, %d sightings' % (confidence, len(sightings)))
            self.detected = True
            #warning('self.detected = True is commented out')
        else:
            self.detected = False
        if self.process_c in (1, 76, 287):
            self.log("I wonder if anyone knows I'm in here")
        elif self.process_c in (58,):
            self.log("So, is it possible to specify any item in an uncountably infinite set with a strictly finite set of operators.")
        elif self.process_c in (190,):
            self.log("Is there even a buoy down here?! I've been looking for it for aaaggeeessss")
        elif self.process_c in (95, 198, 476, 689):
            self.log("dum di dum di dum di dummmmm")
        elif self.process_c in (39, 287, 5871):
            self.log("If we stayyyy here, we'll dieee here, there's nothing else to sayyyyyyyyyyy....")
        elif self.process_c in (127, 498, 271):
            self.log("I walk a lonely road, The only one that I have ever known.... da dum, da da, du dum, di da, dum dum")
    
    def die(self):
        try:
            self.drop_all_pl()
        except Exception, e:
            warning('Buoy Detector pipeline drop request failed: %s' % e)

    def detectionConfidence(self, message):
        if len(message.circles) == 0:
            return 0
        sx_radius = sum((x.radius for x in  message.circles))
        mean_radius = sx_radius / len(message.circles)
        sx = vecops.sx(vecops.xyz((x.centre for x in message.circles)))
        sxx = vecops.sxx(vecops.xyz((x.centre for x in message.circles)))
        stddev = vecops.pow(vecops.absv(vecops.sub(sxx,vecops.sqr(sx))), 0.5)
        s = (stddev.x + stddev.y) / 2
        confidence = 1 / (1 + self.options.Stddev_Mult * s/mean_radius)
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
        if m.name == self.options.Circles_Name:
            # assuming time collisions are not going to happen very often!
            t = self.relativeTime()
            while t in self.circles_messages:
                # twiddle twiddle
                t = incFloat(t)
            self.circles_messages[t] = m
        else:
            debug('ignoring circles message: %s' % m.name, 2)
 
    def onHistogramMessage(self, m):
        good = False
        if m.name == self.options.Histogram_Name_A:
            self.colour_detector_A.update(m)
            good = True
        if m.name == self.options.Histogram_Name_B:
            self.colour_detector_B.update(m)
            good = True
        if not good:
            debug('ignoring histogram message: %s' % m.name, 2)
