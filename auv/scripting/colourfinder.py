import cauv.messaging as messaging

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
