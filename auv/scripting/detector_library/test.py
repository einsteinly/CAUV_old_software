from AI_classes import aiDetector

import time

class detector(aiDetector):
    def __init__(self, node):
        aiDetector.__init__(self, node)
        self.request_pl('broadcast_cams.pipe')
        time.sleep(2)
        
    def process(self):
        #This is an example of what NOT to put in a process command, as this holds up the entire detector_process, preventing everything else from happening.
        if raw_input('Has something been detected?'):
            self.detected = True
