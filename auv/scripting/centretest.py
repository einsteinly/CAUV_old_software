import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node

import time

class CentreTest(messaging.BufferedMessageObserver):

    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("processing")
        node.addObserver(self)

    def onCentreMessage(self, m):
        print 'name: %s x: %f, y: %f' %(m.name, m.x, m.y)

if __name__ == '__main__':
    node = cauv.node.Node('CentreTest')
    auv = control.AUV(node)
    f = CentreTest(node)
    while True:
        time.sleep(5)
