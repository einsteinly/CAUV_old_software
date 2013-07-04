#!/usr/bin/env python2.7
from time import sleep, time
from math import cos, sin, radians

from cauv.debug import debug, warning, info, error
from cauv import messaging

import AI

from utils.boundedtypes import Bearing, MotorValue

#start
#dive
#data timeout
#movement timeout

class dfu(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            #
            self.UsePushStart = True
            self.PushStartDepth = 0.5
            self.Depth = 1
            #
            self.DataTimeout = 2
            self.MinProgress = 0.5
            self.MinProgressSamples = 5
            #
            self.FirstHeading = AI.OptionWithMeta(90, units = " degrees", opt_type=Bearing)
            self.TurnDistance = AI.OptionWithMeta(25, docstring="Distance from wall at which to turn",
                                                    units="m")
            self.TDError = AI.OptionWithMeta(1, docstring="Acceptable error in distance",
                                                    units="m")
            self.TDProportion = 5
            self.TDClamp = AI.OptionWithMeta(30, docstring="Highest speed at which to travel (pre-turning)",
                                                    opt_type=MotorValue)
            self.TDTimeout = AI.OptionWithMeta(0, docstring="Time to travel before giving up.",
                                                    units="s")
            #
            self.SecondHeading = AI.OptionWithMeta(0, units = " degrees", opt_type=Bearing)
            self.SecondSpeed = AI.OptionWithMeta(100, docstring="Speed to travel after turning",
                                                    opt_type=MotorValue)
            self.SecondTime = AI.OptionWithMeta(10, docstring="Time to travel, after turning",
                                                    units="s")
            #
            self.ThirdHeading = AI.OptionWithMeta(180, units = " degrees", opt_type=Bearing)
            self.StopDistance = AI.OptionWithMeta(2, docstring="Distance from back wall at which to finally stop.",
                                                    units="m")
            self.SDError = AI.OptionWithMeta(1, docstring="Acceptable error in distance.",
                                                    units="m")
            self.SDProportion = 5
            self.SDClamp = AI.OptionWithMeta(30, docstring="Highest speed at which to travel in final section.",
                                                    opt_type=MotorValue)
            self.SDTimeout = AI.OptionWithMeta(0, docstring="Time to travel before giving up.",
                                                    units="s")
            
    class Debug(AI.Script.Debug):
        def __init__(self):
            self.stage = 0
            self.distance = None
            self.last_time = 0
            
    def parallel_dist(self, position):
        """
        Calculate length of part of vector to point parallel to direction AUV facing
        """
        n = position.value.north
        e = position.value.east
        heading_n = cos(radians(self.auv.current_bearing))
        heading_e = sin(radians(self.auv.current_bearing))
        return n*heading_n+e*heading_e
        
    def onRelativePositionMessage(self, m):
        if m.origin != "AUV":
            return
        if self.auv.current_bearing == None:
            warning("No bearing information, not calculating distance")
            return
        #depending at which stage we are at we are reporting different distances
        if m.object == "NorthWall" and self.stage == 0:
            self.distance = self.parallel_dist(m.position)
            self.debug.distance = self.distance
            self.last_time = time()
        elif m.object == "BackWall" and self.stage == 2:
            self.distance = self.parallel_dist(m.position)
            self.debug.distance = self.distance
            self.last_time = time()
            
    def wait_for_push(self):
        """
        Wait until we are below a certain depth
        """
        while self.auv.current_depth < self.options.PushStartDepth:
            sleep(0.1)
            
    def head_to_distance(self, target_distance, target_error, proportion, clamp, timeout):
        """
        Head forward until self.distance is within target_error of target_distance
        """
        #make sure clean feed of data
        self.distance = None
        self.last_time = time()
        start_time = time()
        while time()-start_time<timeout or timeout<=0:
            #Data timeout check
            if time()-self.last_time > self.options.DataTimeout:
                error("Data timeout: have not recieved lines for {}s".format(self.options.DataTimeout))
                self.auv.prop(0)
                raise Exception("Data timeout: have not recieved lines for {}s".format(self.options.DataTimeout))
            #Distance exists check
            if self.distance == None:
                sleep(0.1)
                continue
            #Movement check
            #TODO
            #Distance check
            if abs(self.distance-target_distance)<target_error:
                info("Close enough")
                self.auv.prop(0)
                break
            #set speed
            p_speed = (self.distance-target_distance)*proportion
            if p_speed>clamp:
                p_speed=clamp
            elif p_speed<-clamp:
                p_speed=-clamp
            self.auv.prop(int(p_speed))
            sleep(0.1)
        else:
            #didn't break from being close enough, so raise error
            error("Timed out heading to distance.")
            raise Exception("Timed out")
        
    def run(self):
        try:
            if self.options.UsePushStart:
                self.wait_for_push()
            self.auv.depthAndWait(self.options.Depth)
            #STAGE 0
            self.distance = None
            self.stage = 0
            info("Stage 0: Setting initial heading.")
            self.auv.bearingAndWait(self.options.FirstHeading)
            info("Stage 0: Starting to react to walls.")
            self.node.subMessage(messaging.RelativePositionMessage())
            info("Stage 0: Moving forward")
            self.head_to_distance(self.options.TurnDistance, self.options.TDError, self.options.TDProportion, self.options.TDClamp, self.options.TDTimeout)
            #STAGE 1
            self.stage=1
            info("Stage 1: Turning")
            self.auv.bearingAndWait(self.options.SecondHeading)
            info("Stage 1: Moving forward.")
            self.auv.prop(self.options.SecondSpeed)
            sleep(self.options.SecondTime)
            info("Stage 1: Stopping.")
            self.auv.prop(0)
            #STAGE 2
            self.stage = 2
            info("Stage 2: Setting heading.")
            self.auv.bearingAndWait(self.options.ThirdHeading)
            info("Stage 2: Moving forward")
            self.head_to_distance(self.options.StopDistance, self.options.SDError, self.options.SDProportion, self.options.SDClamp, self.options.SDTimeout)
        finally:
            self.auv.depth(0)

Script = dfu

if __name__ == "__main__":
    dfu.entry()        