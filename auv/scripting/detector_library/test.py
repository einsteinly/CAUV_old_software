#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import time

from AI.base.detector import aiDetector, aiDetectorOptions

class detectorOptions(aiDetectorOptions):
    test_value = "HELLO"

class detector(aiDetector):
    def __init__(self, node, opts):
        aiDetector.__init__(self, node, opts)
        self.request_pl('broadcast_cams.pipe')
        time.sleep(2)
        
    def process(self):
        #This is an example of what NOT to put in a process command, as this holds up the entire detector_process, preventing everything else from happening.
        print self.options.test_value
        if raw_input('Has something been detected?'):
            self.detected = True
