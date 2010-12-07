import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node

import time

class ColourFinder(messaging.BufferedMessageObserver):
    def __init__(self, node, bin):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("processing")
        node.addObserver(self)
        self.bin = bin

    def onHistogramMessage(self, m):
        if m.bins[self.bin] > 0.25:
            print "Bin %d is big" % self.bin
        accum = 0
        for i, bin in enumerate(m.bins):
            accum += bin
            print 'bin %d: %f, accum: %f' %(i, bin, accum)

if __name__ == '__main__':
    node = cauv.node.Node('ColFind')
    auv = control.AUV(node)
    detect = ColourFinder(node, 14)
    while True:
        time.sleep(5)
