#!/usr/bin/env python2.7
from time import sleep
from math import cos, sin, radians

from cauv.debug import debug, warning, info, error
from cauv import messaging

import AI

from utils.boundedtypes import Bearing, MotorValue

class dfu(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.Initial_Heading = AI.OptionWithMeta(90, units = " degrees", opt_type=Bearing)
            self.Turn_Distance = AI.OptionWithMeta(20, docstring="Distance from wall at which to turn",
                                                    units="m")
            self.Turn_Distance_Error = AI.OptionWithMeta(1, docstring="Acceptable error in distance",
                                                    units="m")
            self.TDProportion = 5
            self.TDClamp = AI.OptionWithMeta(30, docstring="Highest speed at which to travel (pre-turning)",
                                                    opt_type=MotorValue)
            self.DataTimeout = 2 #TODO use this
            self.Second_Heading = AI.OptionWithMeta(0, units = " degrees", opt_type=Bearing)
            self.Second_Speed = AI.OptionWithMeta(100, docstring="Speed to travel after turning",
                                                    opt_type=MotorValue)
            self.Second_Time = AI.OptionWithMeta(10, docstring="Time to travel, after turning",
                                                    units="s")
            
    class Debug(AI.Script.Debug):
        def __init__(self):
            self.distance = None
        
    def onRelativePositionMessage(self, m):
        if m.origin != "AUV" or m.object != "NorthWall":
            return
        if self.auv.current_bearing == None:
            warning("No bearing information, not calculating distance")
            return
        n = m.position.value.north
        e = m.position.value.east
        heading_n = cos(radians(self.auv.current_bearing))
        heading_e = sin(radians(self.auv.current_bearing))
        perp_dist = n*heading_n+e*heading_e
        self.distance = perp_dist
        self.debug.distance = self.distance
        
    def run(self):
        self.distance = None
        self.stage = 0
        self.auv.bearingAndWait(self.options.Initial_Heading)
        self.node.subMessage(messaging.RelativePositionMessage())
        while self.distance == None:
            sleep(0.1)
        while self.stage == 0:
            if abs(self.distance-self.options.Turn_Distance)<self.options.Turn_Distance_Error:
                self.auv.prop(0)
                self.auv.bearingAndWait(self.options.Second_Heading)
                self.stage=1
            p_speed = (self.distance-self.options.Turn_Distance)*self.options.TDProportion
            if p_speed>self.options.TDClamp:
                p_speed=self.options.TDClamp
            elif p_speed<-self.options.TDClamp:
                p_speed=-self.options.TDClamp
            self.auv.prop(int(p_speed))
            sleep(0.1)
        self.auv.prop(self.options.Second_Speed)
        sleep(self.options.Second_Time)
        self.auv.prop(0)

Script = dfu

if __name__ == "__main__":
    dfu.entry()        