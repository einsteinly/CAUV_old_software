import cauv.messaging as messaging
import cauv.node

import time
import cPickle

class aiMessageListener(messaging.BufferedMessageObserver):
    def __init__(self):
        messaging.BufferedMessageObserver.__init__(self)
        self.node = cauv.node.Node("pymsglst")
        self.node.join("ai")
        self.node.addObserver(self)
    def onAIMessage(self, m):
        print cPickle.loads(m.msg)
        
if __name__ == '__main__':
    aml = aiMessageListener()
    while True:
        time.sleep(100)
        print 'alive'
