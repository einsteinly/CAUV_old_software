#!/usr/bin/env python

import cauv
import cauv.messaging as msg
from cauv.debug import debug, info, warning, error 
from AI_classes import aiScript, aiScriptOptions
from utils.control import expWindow, PIDController

import time
import math
import traceback


class scriptOptions(aiScriptOptions):
    optionName = 4 # for example
    wallDistance = 2 #distance from wall
    strafeSpeed = 5 #controls strafe speed
    strafekPID = (1, 0, 0)
    depthkPID = (1, 0, 0)
    
    class Meta:
        # list of options that can be changed while the script is running
        dynamic = ['optionName']


class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts)
        # self.node is set by aiProcess (base class of aiScript)
        # self.auv is also available, and can be used to control the vehicle
        # self.options is a copy of the option structure declared above
        self.node.join('processing')
    
    def optionChanged(self, option_name):
        info('notified that %s changed to %s' % (option_name[0], option_name[1]))
   
    def onSonarDataMessage(self, m):
        debug('received sonar data: %s' % str(m))

    def run(self):
        info('Wall tracking starting...')
        exit_status = 'SUCCESS'
        try:
            # main loop:
            # ...
            # do stuff!

            time.sleep(0.5) # wait for 0.5 seconds, let other things run
        except Exception, e:
            error(traceback.format_exc())
        finally:
            self.auv.stop()
        info('Complete!')
        # tell the AI framwork that everything went ok (or didn't)
        self.notify_exit(exit_status)
