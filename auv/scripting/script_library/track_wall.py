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
    wallDistance = 2000 #distance from wall in mm 
    strafeSpeed = 10 #controls strafe speed, int [-127, 127]
    wallDistancekPID = (0.05, 0, 0)
    depth = 0 #depth in metres
    runTime = 30 #run time in seconds
    sonarDirection = 180 #in degrees
    sonarWidth = 0 #in 1/6400 of a circle
    sonarGain = 200 #gain is an unsigned byte
    sonarRange = 10000 #in mm
    sonarRangeRes = 100 #in mm
    sonarAngularRes = 1
    doPropLimit = 20 #controls prop limit for dist adjustment, int [-127, 127]
    
    class Meta:
        # list of options that can be changed while the script is running
        dynamic = [
                'strafeSpeed',
                'wallDistancekPID',
                'depth',
                'sonarDirection',
                'sonarGain',
                'sonarRange',
                'sonarRangeRes',
                'doPropLimit'
                ]


class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts)
        # self.node is set by aiProcess (base class of aiScript)
        # self.auv is also available, and can be used to control the vehicle
        # self.options is a copy of the option structure declared above
        self.node.join('processing')
        self.__wallDistance = self.options.wallDistance
        self.__strafeSpeed = self.options.strafeSpeed
        self.wallPID = PIDController(self.options.wallDistancekPID)
        self.__depth = self.options.depth
        self.__runtTime = self.options.runTime
        self.auv.sonar.directionDegrees(self.options.sonarDirection)
        self.auv.sonar.width(self.options.sonarWidth)
        self.auv.sonar.gain(self.options.sonarGain)
        self.auv.sonar.range(self.options.sonarRange)
        self.auv.sonar.rangeRes(self.options.sonarRangeRes)

    def reloadOptions(self):
        self.__strafeSpeed = self.options.strafeSpeed
        self.wallPID = self.wallPID.setKpid(self.options.wallDistancekPID)
        self.__depth = self.options.depth
        self.auv.sonar.directionDegrees(self.options.sonarDirection)
        self.auv.sonar.gain(self.options.sonarGain)
        self.auv.sonar.range(self.options.sonarRange)
        self.auv.sonar.rangeRes(self.options.sonarRangeRes)
    
    def optionChanged(self, option_name):
        info('notified that %s changed to %s' % (option_name[0], option_name[1]))
        self.reloadOptions()
   
    def onSonarDataMessage(self, m):
        #TODO message received?
        debug('received sonar data: %s' % str(m))
        maxIndex = 0
        if m.SonarDataLine.range != 0:
            maxData = 0
            for index, intensity in enumerate(m.SonarDataLine.data):
                if maxData < intensity:
                    maxData = intensity
                    maxIndex = index
        distanceToWall = maxIndex * self.auv.sonar._rangeRes
        debug('Wall at %s mm' % distanceToWall)
        self.actOnDistance(distanceToWall)

    def actOnDistance(self, distance):
        distanceError = self.__wallDistance - distance
        doProp = max([-self.options.doPropLimit, 
            min([self.options.doPropLimit,
                self.wallPID.update(distanceError)])])
        self.auv.prop(int(round(doProp)))
        debug('distance (e=%.3g, ie=%.3g, dg=%.3g)' % (
            self.wallPID.err,
            self.wallPID.ierr,
            self.wallPID.derr
            ))
        debug('prop to %s' % doProp)

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
