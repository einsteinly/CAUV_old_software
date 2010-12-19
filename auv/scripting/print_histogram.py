import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node

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
        for i, bin in enumerate(m.bins):
            accum += bin
            print 'bin %d: %f, accum: %f' %(i, bin, accum)    



    def onHistogramMessage(self, m):
        if m.type == self.channel:

            self.print_bins(m)
            
if __name__ == '__main__':
    node = cauv.node.Node('ColFind')
    auv = control.AUV(node)
    yellowFinder = ColourFinder(node, [11, 12])
    while True:
        time.sleep(5)
