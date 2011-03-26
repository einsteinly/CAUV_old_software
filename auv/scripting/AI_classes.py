import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import threading
import cPickle

#ai messages are of form message is (to, from, function_name, args)
class ai_process(messaging.BufferedMessageObserver):
    def __init__(self, name, process_initial):
        messaging.BufferedMessageObserver.__init__(self)
        self.node = cauv.node.Node(name)
        node.join("ai")
        node.addObserver(self)
        self.process_initial = process_initial
        self.messages = Queue.Queue()
    def onAIMessage(self, m):
        try:
            if m.string[7] == self.process_initial: #this is where the to string appears in the cpickle output
                message = cPickle.loads(m.string)
                self.__getattr__(message[2])(*message[3])
        except:
            debug("Invalid AI message received")
