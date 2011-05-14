import cauv.node
import cauv.control as control
from cauv.debug import debug, warning, error, info

import time

from AI_classes import aiProcess, external_function

@external_function
class auvFunction():
    def __init__(self, auv, func):
        self.func = func
        self.auv = auv
    def __call__(self, *args, **kwargs):
        getattr(self.auv, self.func)(*args, **kwargs)

class auvControl(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'auv_control')
        self.auv = control.AUV(self.node)
        self.external_functions = []
    def __getattr__(self, attr):
        #note python calls tries to get attributes of this class before __getattr__
        return auvFunction(self.auv, attr)
    def run(self):
        while True:
            time.sleep(10)
            #info("auv_control still alive")

if __name__ == '__main__':
    ac = auvControl()
    ac.run()
