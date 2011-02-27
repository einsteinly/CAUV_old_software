import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node
from cauv.debug import debug, warning, error, info
from movingaverage import MovingAverage

import time
import threading

class ColourFinder(messaging.BufferedMessageObserver):
    def __init__(self, node, bin, channel = 'Hue', no_trigger = 3):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("processing")
        node.addObserver(self)
        self.bin = bin
        self.channel = channel
        self.binMovingmean = []

    def print_bins(self, m):
        #Routine to display all the bin values
        accum = 0
        message = ''
        for i, bin in enumerate(m.bins):
            accum += bin
            message += 'bin %d: %f, accum: %f\n' % (i, bin, accum))
        debug(message)

    def onHistogramMessage(self, m):
        if m.name == self.channel:
            self.print_bins(m)
            
if __name__ == '__main__':
    node = cauv.node.Node('ColFind')
    auv = control.AUV(node)
    yellowFinder = ColourFinder(node, [11, 12])
    while True:
        time.sleep(5)
