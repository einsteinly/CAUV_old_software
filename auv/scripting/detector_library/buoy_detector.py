#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import time
import sys
import math

import cauv.pipeline as pipeline
from cauv.messaging import floatXYZ
from cauv.debug import debug, info, warning, error

from utils import vecops
from utils.hacks import incFloat
from utils.detectors import ColourDetector

import AI

class BuoyDetector(AI.Detector):
    class DefaultOptions(AI.Detector.DefaultOptions):
        def __init__(self):
            self.Sightings_Period = 5.0 # seconds, period to consider sightings of the buoy for
            self.Required_Confidence = 0.9
            self.Required_Sightings = 5
            self.Circles_Name = 'buoy'
            self.Histogram_Name_A = 'buoy_hue'
            self.Histogram_Name_B = 'buoy_hue'
            self.Histogram_Bins_A = xrange(60,120) # of 0--180 range
            self.Histogram_Bins_B = xrange(50,75) # of 0--180 range
            self.Stddev_Mult = 0.2 # lower --> higher confidence when multiple circles with different centres are visible
            self.Optimal_Colour_Frac_A = 0.003 # highest colour detection confidence when the specified bin contains this value
            self.Optimal_Colour_Frac_B = 0.008 # highest colour detection confidence when the specified bin contains this value
            self.Colour_Weight_A  = -20.0 # respective weightings in confidence
            self.Colour_Weight_B  =  0.4 #
            self.Circles_Weight   =  1.6 #
            self.Process_Frequency = AI.OptionWithMeta(0.5, dynamic=True, docstring="Frequency of processing image pipeline data.")
            self.Fire_Time = AI.OptionWithMeta(0.5, dynamic=True, docstring="How long the detector stays on after finding the buoy.")
            
    class Debug(AI.Detector.Debug):
        def __init__(self):
            self.confidence = 0
            self.circles_confidence = 0
            self.colour_confidence_A = 0
            self.colour_confidence_B = 0
            
    def __init__(self):
        AI.Detector.__init__(self)
        self.load_pipeline('detect_buoy_sim')
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
        self.log('Looking for the buoy')
        self.process_c = 0
    
    def relativeTime(self):
        return time.time() - self.tzero

    def run(self):
        while True:
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
                self.circles_confidence = self.options.Circles_Weight * sum(sightings[:numcircles]) / numcircles
            else:
                self.circles_confidence = 0
            self.colour_confidence_A = self.options.Colour_Weight_A * self.colour_detector_A.confidence()
            self.colour_confidence_B = self.options.Colour_Weight_B * self.colour_detector_B.confidence()
            self.confidence = self.circles_confidence + self.colour_confidence_A + self.colour_confidence_B
            info('buoy detector processing %d sightings, confidence=%g (circles=%g,colour_A=%g,colour_B=%g)' % (
                len(sightings), self.confidence, self.circles_confidence, self.colour_confidence_A, self.colour_confidence_B
            ))
            debug('mean frac: A=%g B=%g' % (self.colour_detector_A.frac(), self.colour_detector_B.frac()))
            if self.confidence >= self.options.Required_Confidence and \
            len(sightings) >= self.options.Required_Sightings:
                info('buoy detected, confidence = %g, %d sightings' % (self.confidence, len(sightings)))
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
                
            time.sleep(self.options.Process_Frequency)
    
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
            
Detector = BuoyDetector

if __name__ == "__main__":
    BuoyDetector.entry()