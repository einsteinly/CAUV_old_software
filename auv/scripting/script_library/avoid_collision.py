#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.node
from cauv.debug import debug, info, warning, error

from AI_classes import aiScript

class AvoidCollisionOptions:
    Node_Name = "py-collision-avoid"
    
class script(aiScript):
    def __init__(self):
        aiScript.__init__(self, AvoidCollisionOptions.Node_Name)
        # self.node is set by aiProcess (base class of aiScript)
        self.node.join('processing')
    
    def run(self):
        print 'About to hit something! reversing for 5 seconds'
        auv.prop(-127)
        time.sleep(5)
