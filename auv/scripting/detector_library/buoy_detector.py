from AI_classes import aiDetector
from cauv.debug import debug, info, warning, error

import cauv.pipeline as pipeline

import time
import sys
import math

class BuoyDetectorOptions:
    Sightings_Period   = 5.0 # seconds, period to consider sightings of the buoy for
    Required_Confidence = 0.9 # required proportion of CirclesMessages with qualifying sightings
    Pipeline_File = 'pipelines/detect_buoy.pipe'
    Load_Pipeline = 'buoy-detect'
    Circles_Name = 'buoy'

class detector(aiDetector):
    def __init__(self, node):
        aiDetector.__init__(self, node)
        self.circles_messages = {} # map time received : message
        self.tzero = time.time()
        self.node.join('processing')
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
        for t, m in  self.circles_messages.items():
            sightings.append(self.detectionConfidence(m))
        if len(sightings):
            confidence = sum(sightings) / len(sightings)
        else:
            confidence = 0
        info('buoy detector processing %d sightings' % len(sightings))
        if confidence > BuoyDetectorOptions.Required_Confidence:
            info('buoy detected, confidence = %g, %d sightings' % (confidence, len(sightings)))
            self.detected = True
        else:
            info('buoy not detected, detection confidence = %g, %d sightings' % (confidence, len(sightings)))
            self.detected = False
    
    def die(self):
        # save the running pipeline in case someone edited it!
        if BuoyDetectorOptions.Load_Pipeline is not None:
            info('saving the current %s pipeline to %s' % (BuoyDetectorOptions.Load_Pipeline,
                                                           BuoyDetectorOptions.Pipeline_File))
            self.__pl.load(BuoyDetectorOptions.Pipeline_File + '.autosaved')

    def detectionConfidence(self, message):
        # TODO: something more sophisticated?
        if len(message.circles) >= 1:
            return 1.0
        return 0.0

    def cullOldSightings(self, cull_before_time):
        to_remove = []
        for t in self.circles_messages:
            if t < cull_before_time:
                to_remove.append(t)
        for t in to_remove:
            del self.circles_messages[t]

    def onCirclesMessage(self, m):
        if m.name == BuoyDetectorOptions.Circles_Name:
            # assuming time collisions are not going to happen very often!
            t = self.relativeTime()
            while t in self.circles_messages:
                # twiddle twiddle
                m, e = math.frexp(t)
                t = (m + sys.float_info.epsilon) * 2**e
            self.circles_messages[t] = m
        else:
            debug('ignoring circles message: %s' % m.name)
