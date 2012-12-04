from cauv.debug import debug, info, error, warning

import time

from AI.base import aiScript

class script(aiScript):
    def run(self):
        self.log('Mission time limit reached, surfacing.')
        self.auv.depthAndWait(0, timeout=10)
        self.ai.auv_control.disable()
        while True:
            info('Surface script still alive, waiting to be manually killed')
            time.sleep(5)
            
