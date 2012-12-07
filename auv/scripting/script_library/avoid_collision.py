#!/usr/bin/env python2.7
import time

from cauv.debug import debug, info, warning, error

from AI.base.script import aiScript, aiScriptOptions

class scriptOptions(aiScriptOptions):
    class Meta:
        dynamic = []

class script(aiScript):    
    def run(self):
        print 'About to hit something! reversing for 5 seconds'
        self.auv.stop()
        self.auv.prop(-127)
        time.sleep(5)
