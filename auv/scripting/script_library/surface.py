from cauv.debug import debug, info, error, warning

import time

import AI

class Surface(AI.Script):
    def run(self):
        self.log('Mission time limit reached, surfacing.')
        self.auv.depthAndWait(0, timeout=10)
        self.ai.auv_control.disable()
        while True:
            info('Surface script still alive, waiting to be manually killed')
            time.sleep(5)
            

Script = Surface

if __name__ == "__main__":
    Surface.entry()