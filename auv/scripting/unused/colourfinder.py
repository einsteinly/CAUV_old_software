#!/usr/bin/env python2.7

import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node
from cauv.debug import debug, warning, error, info
from movingaverage import MovingAverage

import time
import threading

class ColourFinder(messaging.BufferedMessageObserver):
    def __init__(self, node, bin, no_trigger = 3, tolerance = 0.15, maxcount=30, st_multiplier=3, st_on=1):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("processing")
        node.addObserver(self)
        self.bin = bin
        self.no_trigger = no_trigger
        self.detect = 0
        self.binMovingmean = []
        self.lock = threading.Lock()
        for uni_bin in self.bin:
            # A class to calculate the moving average of last maxcount number of
            # sample, multiplier sets the tolerance range multiplier and set
            # trigger flag when sample is outside tolerance range:
            self.binMovingmean.append(MovingAverage('upper', tolerance, maxcount, st_multiplier, st_on))

    def print_bins(self, m):
        #Routine to display all the bin values
        accum = 0
        message = ''
        for i, bin in enumerate(m.bins):
            accum += bin
            message += 'bin %d: %f, accum: %f\n' % (i, bin, accum)
        debug(message)

    def detected(self):
        self.lock.acquire()
        r = self.detect
        self.lock.release()
        return r

    def onHistogramMessage(self, m):
        #self.print_bins(m)
        for j, uni_bin in enumerate(self.bin):
            self.binMovingmean[j].update(m.bins[uni_bin])
            #print 'bin %d: %f' %(uni_bin, m.bins[uni_bin])
            if (self.binMovingmean[j].trigger > self.no_trigger):
                self.lock.acquire()
                self.detect = 1
                self.lock.release()
                info("Looks like we have found something!!!")
                break
            else:
                self.lock.acquire()
                self.detect = 0
                self.lock.release()

            debug('Count : %d' % self.binMovingmean[j].count)
            debug('Moving average of bin %d is: %f' %(uni_bin, self.binMovingmean[j].movingMean))
            debug('Standard error of bin %d is: %f' %(uni_bin, self.binMovingmean[j].movingError))

if __name__ == '__main__':
    node = cauv.node.Node('ColFind')
    auv = control.AUV(node)
    yellowFinder = ColourFinder(node, [11, 12], 3, 0.15, 30, 3, 1)
    while True:
        time.sleep(1.0)
