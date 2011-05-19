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
        message = cPickle.loads(m.msg)
        print '%s: %s.%s(%s, %s)' %(message[1],message[0],message[2],', '.join(map(str,message[3])),', '.join(['='.join(map(str, x)) for x in message[4].items()]))
        
if __name__ == '__main__':
    aml = aiMessageListener()
    while True:
        time.sleep(100)
        print 'alive'
