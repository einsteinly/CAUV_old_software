#!/usr/bin/env python2.7
import cauv
import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import os
import yaml
import time
import shelve
import socket
import argparse
import datetime
import traceback
import itertools
import collections

from collections import deque
from math import degrees, atan, cos, sin, radians

import utils.event as event

LINES_NAME = "wall_lines"
LINES_NAME_SCALE = "wall_lines_scale"
ARENA_BEARING_CORRECTION = -90 #degrees, bearing of north on map below, competition: 17, simulator: -90
TOLERANCE = 5 #degrees

class LocationManager(event.EventLoop, messaging.MessageObserver):
    #SETUP FUNCTIONS
    def __init__(self):
        super(LocationManager, self).__init__()
        self.node = cauv.node.Node("location_manager")
        #subscribe to appropriate messages
        self.node.subMessage(messaging.LinesMessage())
        self.node.subMessage(messaging.FloatMessage())
        self.node.subMessage(messaging.TelemetryMessage())
        #setup appropriate image pipeline
        #setup initial values
        self.bearing = None # (corrected) bearing
        self.image_scale = None # width of image
        self.node.addObserver(self)

    #wall position stuff

    @event.event_func
    def onTelemetryMessage(self, m):
        self.real_bearing = m.orientation.yaw
        self.bearing = (self.real_bearing + ARENA_BEARING_CORRECTION)%360
        
    @event.event_func
    def onFloatMessage(self, m):
        if m.name != LINES_NAME_SCALE:
            return
        self.image_scale = m.value

    @event.event_func
    def onLinesMessage(self, m):
        if m.name != LINES_NAME:
            return
        if self.bearing == None:
            warning("No orientation information, not processing lines")
            return
        if self.image_scale == None:
            warning("No scale information, not processing lines")
            return
        #categorise lines
        #        -------------|North wall
        #                     |
        #                     |Back/east wall      N
        #                     |                   W E
        #                     |                    S
        #        -------------|South wall
        north_wall = []
        back_wall = []
        south_wall = []
        other_lines = []
        for line in m.lines:
            line_bearing = (degrees(line.angle)+self.bearing)%180
            #if the wall is east-west it is a side wall
            if line_bearing<=90+TOLERANCE and line_bearing>=90-TOLERANCE:
                #4 situations, assume North wall is exactly to North then, since degree of sonar is 120:
                #-30<bearing<30 => North wall
                #30<bearing<150 => North left, South right
                #150<bearing<210 => South wall
                #210<bearing<330 = > North right, South left, no back wall
                if self.bearing > 330 or self.bearing < 30:
                    north_wall.append(line)
                elif self.bearing > 150 and self.bearing < 210:
                    south_wall.append(line)
                else:
                    #calculate whether the line is left or right
                    #calculate angle to centre, between +-90
                    centre_angle = degrees(atan((line.centre.y-0.5)/(line.centre.x-0.5)))
                    #if angle to centre < angle of line (in range +- 90), then line is to left
                    if (centre_angle < (degrees(line.angle)+90)%180-90)^(self.bearing>210 and self.bearing<330):
                        north_wall.append(line)
                    else:
                        south_wall.append(line)
            #if it is north-south it is the back wall (hopefully, maybe add a check?)
            elif line_bearing<=TOLERANCE or line_bearing>=180-TOLERANCE:
                back_wall.append(line)
            else:
                other_lines.append(line)
        #warn or ignore if other_lines to high?
        info("%d north wall, %d back wall, %d south wall, %d other lines" %(len(north_wall), len(back_wall), len(south_wall), len(other_lines)))
        if north_wall:
            rel_x = sum([line.centre.x-0.5 for line in north_wall])*self.image_scale/len(north_wall)
            rel_y = sum([line.centre.y-0.5 for line in north_wall])*self.image_scale/len(north_wall)
            north = rel_x*cos(radians(self.real_bearing))+rel_y*sin(radians(self.real_bearing))
            east = rel_x*sin(radians(self.real_bearing))-rel_y*cos(radians(self.real_bearing))
            self.node.send(messaging.RelativePositionMessage('north_wall', 'auv', messaging.CartesianPosition2D(north, east, 0)))
        if south_wall:
            rel_x = sum([line.centre.x-0.5 for line in south_wall])*self.image_scale/len(south_wall)
            rel_y = sum([line.centre.y-0.5 for line in south_wall])*self.image_scale/len(south_wall)
            north = rel_x*cos(radians(self.real_bearing))+rel_y*sin(radians(self.real_bearing))
            east = rel_x*sin(radians(self.real_bearing))-rel_y*cos(radians(self.real_bearing))
            self.node.send(messaging.RelativePositionMessage('south_wall', 'auv', messaging.CartesianPosition2D(north, east, 0)))
        if back_wall:
            rel_x = sum([line.centre.x-0.5 for line in back_wall])*self.image_scale/len(back_wall)
            rel_y = sum([line.centre.y-0.5 for line in back_wall])*self.image_scale/len(back_wall)
            north = rel_x*cos(radians(self.real_bearing))+rel_y*sin(radians(self.real_bearing))
            east = rel_x*sin(radians(self.real_bearing))-rel_y*cos(radians(self.real_bearing))
            self.node.send(messaging.RelativePositionMessage('back_wall', 'auv', messaging.CartesianPosition2D(north, east, 0)))

if __name__ == '__main__':
    lm = LocationManager()
    lm.run()