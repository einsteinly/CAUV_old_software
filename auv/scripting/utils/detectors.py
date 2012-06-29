import time
import threading
from utils.hacks import incFloat

class ColourDetector:
    def __init__(self, sightings_period, bin_num = None, optimal_colour_frac = 0.1, bins = []):
        self.optimal_colour_frac = optimal_colour_frac
        self.sightings_period    = sightings_period
        self.max_frac = 1.0
        self.tzero = time.time()
        self.histogram_messages = {}
        self.bins = list(bins)
        if bin_num is not None:
            self.bins.append(bin_num)

    def relativeTime(self):
        return time.time() - self.tzero

    def colourFracConfidence(self, frac):
        #  ^
        # 1+      .
        #  |     / \
        #  |   /     \
        #  |  /        \
        #  | /           \
        #  +------^-------+-> frac
        #  0      |        1
        #         | optimal
        #debug('colourFracConfidence: frac=%g' % frac)
        if frac <= self.optimal_colour_frac:
            return frac / self.optimal_colour_frac
        elif frac > self.optimal_colour_frac:
            return (frac - self.optimal_colour_frac) / (self.max_frac - self.optimal_colour_frac)
        else:
            # this case occurs if frac = NaN
            return 0

    def cullOldSightings(self, cull_before_time):
        to_remove = []
        for t in self.histogram_messages.keys(): #call keys to get a list that wont chnage if something is added
            if t < cull_before_time:
                to_remove.append(t)
        for t in to_remove:
            del self.histogram_messages[t]

    def update(self, histogram_msg):
        self.histogram_messages[self.relativeTime()] = histogram_msg

    def frac(self):
        # return mean colour fraction (for information only)
        r = 0
        for t, m in self.histogram_messages.items(): #call items to get an unchangin list
            frac = 0
            for b in self.bins:
                frac += m.bins[b]
            r += frac
        n = len(self.histogram_messages)
        if n > 0:
            r /= n
        return r

    def confidence(self):
        tnow = self.relativeTime()
        self.cullOldSightings(tnow - self.sightings_period)
        sightings = []
        numcolours = 0
        for t, m in self.histogram_messages.items():
            frac = 0
            for b in self.bins:
                frac += m.bins[b]
            sightings.append(self.colourFracConfidence(frac))
            numcolours += 1
        if numcolours:
            colour_confidence = sum(sightings) / numcolours
        else:
            colour_confidence = 0
        return colour_confidence
