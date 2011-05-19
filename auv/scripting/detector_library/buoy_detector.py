from AI_classes import aiDetector

import time

class BuoyDetectorOptions:
    Sightings_Period   = 5.0 # seconds, period to consider sightings of the buoy for
    Required_Confidence = 0.9 # required proportion of CirclesMessages with qualifying sightings

class detector(aiDetector):
    def __init__(self, node):
        aiDetector.__init__(self, node)
        self.circles_messages = {} # map time received : message
        self.tzero = time.time()
        # TODO: load pipeline
    
    def process(self):
        # process recorded observations of circles, and set self.detected True
        # / False as necessary
        tnow = time.time()
        self.cullOldSightings(tnow-BuoyDetectorOptions.Sightings_Period)
        sightings = []
        for t, m in  self.circles_messages.items():
            sightings.append(self.detectionConfidence(m))
        if len(sightings):
            confidence = sum(sightings) / len(sightings)
        else:
            confidence = 0
        if confidence > BuoyDetectorOptions.Required_Confidence:
            self.detected = True
        else:
            self.detected = False
    
    def detectionConfidence(self, message):
        # TODO: something more sophisticated?
        if len(message.circles) >= 1:
            return 1
        return 0

    def cullOldSightings(self, cull_before_time):
        to_remove = []
        for t in self.circles_messages:
            if t < cull_before_time:
                to_remove.append(t)
        for t in to_remove:
            del self.circles_messages[t]

    def onCirclesMessage(self, m):
        # assuming time collisions are not going to happen very often!
        t = time.time() - tzero
        while t in self.circles_messages:
            t += 1e-9
        self.circles_messages[t] = m
