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
    wallDistance = 2 #distance from wall in metres TODO: check units
    strafeSpeed = 5 #controls strafe speed, int [-127, 127]
    wallDistancekPID = (1, 0, 0)
    depth = 0 #depth
    runTime = 30 #run time in seconds
    sonarAngle = 180 #in degrees
    
    class Meta:
        # list of options that can be changed while the script is running
        dynamic = [
                'wallDistancekPID',
                'strafeSpeed',
                'depth',
                'sonarAngle'
                ]


class script(aiScript):
    def __init__(self, script_name, opts):
        #TODO point sonar back
        aiScript.__init__(self, script_name, opts)
        # self.node is set by aiProcess (base class of aiScript)
        # self.auv is also available, and can be used to control the vehicle
        # self.options is a copy of the option structure declared above
        self.node.join('processing')
        self.__wallDistance = self.options.wallDistance
        self.__strafeSpeed = self.options.strafeSpeed
        self.wallPID = PIDController(self.options.wallDistancekPID)
        self.__runtTime = self.options.runTime

    def reloadOptions(self):
        self.__wallDistance = self.options.wallDistance
        self.__strafeSpeed = self.options.strafeSpeed
        self.wallPID = self.wallPID.setKpid(self.options.wallDistancekPID)
    
    def optionChanged(self, option_name):
        info('notified that %s changed to %s' % (option_name[0], option_name[1]))
        self.reloadOptions()
   
    def onSonarDataMessage(self, m):
        #TODO 
        debug('received sonar data: %s' % str(m))

    def run(self):
        info('Wall tracking starting...')
        exit_status = 'SUCCESS'
        time_left = self.__runtTime
        try:
            while time_left > 0:
                self.auv.strafe(self.__strafeSpeed)
                time.sleep(0.5)
                time_left -= 0.5

            # main loop:
            # ...
            # do stuff!

        except Exception, e:
            error(traceback.format_exc())
        finally:
            self.auv.stop()
            info('Stopping')
        info('Complete!')
        # tell the AI framwork that everything went ok (or didn't)
        self.notify_exit(exit_status)
