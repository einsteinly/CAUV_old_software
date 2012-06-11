#!/usr/bin/env python2.7

import cauv
import cauv.messaging as messaging
import cauv.pipeline as pipeline
import cauv.node

from utils.control import PIDController
from utils.timeaverage import TimeAverage
from cauv.debug import debug, info, warning, error
from AI_classes import aiScript, aiScriptOptions

import threading
from math import degrees, cos, sin
import time

class scriptOptions(aiScriptOptions):
    wall_pipeline_file  = 'track_wall.pipe'
    lines_name = 'walls'


class script(aiScript):
    def __init__(self, *args, **kwargs):
        aiScript.__init__(self, *args, **kwargs)
        self.node.join("processing")

    def onLinesMessage(self, m):
        if m.name != self.options.lines_name:
            return
        if len(m.lines):
            debug('got lines: %s' % m.lines)
        else:
            debug('no lines!')
    

    def run(self):
        self.log('wall tracking task started: looking for the wall')
        
        while True:
            time.sleep(0.1)

        return 'SUCCESS'
