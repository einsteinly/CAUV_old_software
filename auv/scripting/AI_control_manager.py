import cauv.node
import cauv.control as control
from cauv.debug import debug, warning, error, info

import time

from AI_classes import aiProcess, external_function

class auvControl(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'auv_control')
        self.auv = control.AUV(self.node)
        self.external_functions = []
    def __getattr__(self, attr):
        #note python calls tries to get attributes of this function before __getattr__
        try:
            return external_function(getattr(self.auv, attr))
        except AttributeError:
            debug("Failed to interpret message to auv control")
    def run(self):
        while True:
            time.sleep(10)
            info("auv_control still alive")

if __name__ == '__main__':
    ac = auvControl()
    ac.run()
