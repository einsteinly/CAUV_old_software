#!/usr/bin/env python2.7

import cauv
import cauv.messaging as msg
from cauv.debug import debug, info, warning, error 
import AI
from utils.control import PIDController
from utils.coordinates import Simulation_Datum, NorthEastDepthCoord

import time
import math
import traceback
import threading
from collections import deque

class InsufficientDataError(ValueError):
    pass

class TrackWallSimple(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.strafekP = 1000 #controls strafe speed, int [-127, 127]
            self.strafeLimit = AI.OptionWithMeta(60, type=MotorValue)
            self.strafeOffset = AI.OptionWithMeta(10, type=MotorValue)
            self.wallDistancekP = -1000
            self.depth = AI.OptionWithMeta(2, units="m") #depth in metres
            self.useDepth = True
            self.maximumRunTime = AI.OptionWithMeta(200, units="s") #run time in seconds
            self.targetDistance = 0.1
            self.controlBearing = False
            self.initialLocation = (Simulation_Datum+NorthEastDepthCoord(-18, 5, 0)).toWGS84()
            self.initialBearing = 80
            self.perpendicularError = 5
            self.perpendicularRationLimit = 0.1
            self.linesName = 'track_wall'
            self.pipeline = 'track_wall2'
    
    class Debug(AI.Script.Debug):
        def __init__(self):
            self.angle = 0
            self.distance = 0
            
    def __init__(self):
        self.angle = 0
        self.distance = 0
        
    def onLinesMessage(self, m):
        if m.name != self.options.linesName:
            return
        """
!!!! Write help here !!!!

        """
        angle, distance = findBestLine(m.lines)
        perpendicularRatio = findPerpendicularPercent();
        
        #set debug values
        self.angle = angle
        self.distance = distance
        debug('Angle and distance are %f, %f' %(angle,distance))
        
        #Prop Control
        speed = 0
        if perpendicularRatio < self.perpendicularRationLimit
            speed = (0.5+self.options.targetDistance-distance)*self.options.wallDistancekP+self.options.strafeOffset
        debug('Setting speed %f' %(speed))
        self.auv.prop(int(speed))
        
        #Bearing Control
        if self.controlBearing
            debug('Setting bearing %f' %(self.auv.current_bearing+angle))
            self.auv.bearing(self.auv.current_bearing+angle)
            
        #Strafe Control
        #don't strafe when to far from the wall because the AUV won't see the wall with its cameras
        strafe_speed = 0
        if perpendicularRatio < self.perpendicularRationLimit
            strafe_speed = (0.5+self.options.targetDistance-distance)*self.options.strafekP
            strafe_speed = self.options.strafeLimit if strafe_speed > self.options.strafeLimit else strafe_speed
            strafe_speed = -self.options.strafeLimit if strafe_speed < -self.options.strafeLimit else strafe_speed
        self.auv.strafe(int(strafe_speed))
        debug('Setting strafe %f' %(strafe_speed))
    
    def findBestLine(self, lines)
        #Just take the weighted(based on line length) average of everything
        length = 0
        angle = 0
        distance = 0
        for line in lines
            #Find distance from the current position to the line.
            #direction of the line
            #IS THE ANGLE IN RADIANS OR DEGREES HERE?
            t = msg.floatXY(math.cos(line.angle), math.sin(line.angle))
            #a vector from the AUV to any point of the line
            a = msg.floatXY(line.centre.x - 0.5, line.centre.y - 0.5)
            distance += math.abs(t.x*a.y - t.y*a.x)*line.length
            angle += line.angle*line.length
            length += line.length
        return angle/length, distance/length;     
        
    def findPerpendicularRatio(self, lines)
        totalPairLength = 0
        perpedicularLength = 0
        for i in range(len(lines))
            for j in range(i:len(lines))
                line1 = line[i]
                line2 = line[j]
                if math.abs(line1.angle-line2.angle) < self.perpendicularError
                    perpedicularLength += line1.length + line2.length
                totalPairLength += line1.length + line2.length
                    
        return perpedicularLength/totalPairLength
             
        
    def run(self):
        #head to start
        time.sleep(2)
        if self.options.useDepth:
            self.auv.depth(self.options.depth)
        #self.auv.headToLocation(self.options.initialLocation)
        self.auv.bearingAndWait(self.options.initialBearing)
        #start onMessage handler
        start_time = time.time()
        self.node.subMessage(msg.LinesMessage())
        #this allows dynamic setting of running time
        while time.time()-start_time < self.options.maximumRunTime:
            time.sleep(1)
            #really should have some other end condition aside from time
            

Script = TrackWallSimple

if __name__ == "__main__":
    TrackWallSimple.entry()
