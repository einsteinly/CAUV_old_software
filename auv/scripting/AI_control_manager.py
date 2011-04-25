import cauv.node
import cauv.control as control
from cauv.debug import debug, warning, error, info

import AI_messages

class auvControl():
    def __init__(self):
        self.node = cauv.node.Node('pyauvcon')
        self.ai_node = AI_messages.ai_node(self.node, process_initial='d')
        self.auv = control.AUV(self.node)
    def run(self):
        while True:
            message = self.ai_node.getMessage(block=True)
            try:
                self.auv.__getattr__(message[2])(*message[3])
            except AttributeError:
                try:
                    self.__getattr__(message[2])(*message[3])
                except AttributeError:
                    debug("Failed to interpret message to auv control")

if __name__ == '__main__':
    ac = auvControl()
    ac.run()
