#!/usr/bin/env python2.7
import time

from cauv.debug import debug, info, warning, error

import AI

class AvoidCollision(AI.Script):                
    def run(self):
        print 'About to hit something! reversing for 5 seconds'
        self.auv.stop()
        self.auv.prop(-127)
        time.sleep(5)
        
Script = AvoidCollision

if __name__ == "__main__":
    AvoidCollision.entry()