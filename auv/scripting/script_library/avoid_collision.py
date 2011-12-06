#!/usr/bin/env python2.7

from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, info, warning, error

import time

class scriptOptions(aiScriptOptions):
    class Meta:
        dynamic = []

class script(aiScript):    
    def run(self):
        print 'About to hit something! reversing for 5 seconds'
        self.auv.stop()
        self.auv.prop(-127)
        time.sleep(5)
