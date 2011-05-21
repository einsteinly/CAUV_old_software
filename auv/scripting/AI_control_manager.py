import cauv.node
import cauv.control as control
from cauv.debug import debug, warning, error, info

import time

from AI_classes import aiProcess, external_function

#TODO basically the actual functionality of conrol, the ability to stop the sub, block script_ids etc

class auvControl(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'auv_control')
        self.auv = control.AUV(self.node)
        self.external_functions = []
    @external_function
    def auv_command(self, script_id, command, *args, **kwargs):
        #__getattr__ was more trouble than its worth. since this is abstracted by fakeAUV, doesn't matter to much
        #TODO make it possible to filter by script id. script id should match task_managers record as well
        #Might need to move parts of control here/take a smaller version of control that doesn't have waiting commands (eg depth and wait)
        #note, we don't care about errors here, cos they'l be caught by the message handler.
        #Also the message handler will tell us which message from who caused the error
        getattr(self.auv, command)(*args, **kwargs)
    @external_function
    def stop(self):
        self.auv.stop()
    def run(self):
        while True:
            time.sleep(10)
            #info("auv_control still alive")

if __name__ == '__main__':
    ac = auvControl()
    ac.run()
