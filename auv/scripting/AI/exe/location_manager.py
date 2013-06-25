#!/usr/bin/env python2.7
import cauv
import cauv.messaging as messaging
import cauv.node
import cauv.yamlpipe
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
from math import degrees, atan, cos, sin, radians, tan

import utils.event as event
import utils.dirs

#TODO this should probably be part of some vector library....
def rotate(vec, angle):
    """
    Rotates the vector by angle degrees counterclockwise
    """
    return (vec[0]*cos(radians(angle))-vec[1]*sin(radians(angle)),
            vec[1]*cos(radians(angle))+vec[0]*sin(radians(angle)))
    
def intersection(line1, line2):
    if line1.angle == line2.angle:
        raise ArithmeticError("Lines are parallel.")
    x1 = line1.centre.x
    y1 = line1.centre.y
    x2 = line2.centre.x
    y2 = line2.centre.y
    tan1 = tan(line1.angle)
    tan2 = tan(line2.angle)
    x = (y2-y1+x1*tan1-x2*tan2)/(tan1-tan2)
    y = (x-x1)*tan1+y1
    return (x,y)

def mean_lines(lines):
    centre = (sum([line.centre.x for line in lines])/len(lines),
              sum([line.centre.y for line in lines])/len(lines))
    angle = sum([(line.angle-lines[0].angle+90)%180])/len(lines)-90+lines[0].angle
    length = max([line.length for line in lines])
    return messaging.Line(messaging.floatXY(*centre), angle, length, lines[0].width)

class LocationManager(event.EventLoop, messaging.MessageObserver):
    #SETUP FUNCTIONS
    def __init__(self, opts):
        super(LocationManager, self).__init__()
        self.node = cauv.node.Node("location_manager")
        #subscribe to appropriate messages
        self.node.subMessage(messaging.LinesMessage())
        self.node.subMessage(messaging.FloatMessage())
        self.node.subMessage(messaging.TelemetryMessage())
        #setup appropriate image pipeline
        pipe_file = os.path.join(utils.dirs.config_dir('pipelines'), opts.pipeline_name) + ".yaml"
        with open(pipe_file) as pf:
            pipeline = cauv.yamlpipe.load(pf)
        model = cauv.pipeline.Model(self.node, 'ai/_wall_lines')
        pipeline.fixup_inputs()
        model.set(pipeline)
        #setup initial values
        self.bearing = None # (corrected) bearing
        self.real_bearing = None # (actual) bearing
        self.image_scale = None # width of sonar image
        self.wall_length = opts.wall_length
        self.tolerance = opts.tolerance
        self.bearing_correction = opts.arena_bearing_correction
        self.scale_name = opts.scale_name
        self.lines_name = opts.lines_name
        self.node.addObserver(self)

    #WALL POSITIONING
    @event.event_func
    def onTelemetryMessage(self, m):
        self.real_bearing = m.orientation.yaw
        self.bearing = (self.real_bearing + self.bearing_correction)%360
        
    @event.event_func
    def onFloatMessage(self, m):
        if m.name != self.scale_name:
            return
        self.image_scale = m.value

    @event.event_func
    def onLinesMessage(self, m):
        if m.name != self.lines_name:
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
            if line_bearing<=90+self.tolerance and line_bearing>=90-self.tolerance:
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
            elif line_bearing<=self.tolerance or line_bearing>=180-self.tolerance:
                back_wall.append(line)
            else:
                other_lines.append(line)
        #warn or ignore if other_lines to high?
        #TODO bearing correction based on walls
        info("%d north wall, %d back wall, %d south wall, %d other lines" %(len(north_wall), len(back_wall), len(south_wall), len(other_lines)))
        #send positions of known walls, origin the auv
        if north_wall:
            north_wall = mean_lines(north_wall)
            rel_n = (north_wall.centre.x-0.5)*self.image_scale
            rel_e = (north_wall.centre.y-0.5)*self.image_scale
            position = rotate((rel_n,rel_e), self.real_bearing)
            self.node.send(messaging.RelativePositionMessage('NorthWall', 'AUV', messaging.CartesianPosition2D(*position)))
        if south_wall:
            south_wall = mean_lines(south_wall)
            rel_n = (south_wall.centre.x-0.5)*self.image_scale
            rel_e = (south_wall.centre.y-0.5)*self.image_scale
            position = rotate((rel_n,rel_e), self.real_bearing)
            self.node.send(messaging.RelativePositionMessage('SouthWall', 'AUV', messaging.CartesianPosition2D(*position)))
        if back_wall:
            back_wall = mean_lines(back_wall)
            rel_n = (back_wall.centre.x-0.5)*self.image_scale
            rel_e = (back_wall.centre.y-0.5)*self.image_scale
            position = rotate((rel_n,rel_e), self.real_bearing)
            self.node.send(messaging.RelativePositionMessage('BackWall', 'AUV', messaging.CartesianPosition2D(*position)))
        #if sufficient information, send position of north east corner of harbour
        if back_wall:
            if north_wall:
                #calculate intersection+scale
                rel_p = intersection(north_wall, back_wall)
                rel_p = ((rel_p[0]-0.5)*self.image_scale, (rel_p[1]-0.5)*self.image_scale)
                #rotate
                pos = rotate(rel_p, self.real_bearing)
                self.node.send(messaging.RelativePositionMessage('NECorner', 'AUV', messaging.CartesianPosition2D(*pos)))
            elif south_wall:
                #calculate intersection + scale
                rel_p = intersection(south_wall, back_wall)
                rel_p = ((rel_p[0]-0.5)*self.image_scale, (rel_p[1]-0.5)*self.image_scale)
                #rotate
                pos = rotate(rel_p, self.bearing)
                #move north by length of wall
                pos = (pos[0]+self.wall_length, pos[1])
                #rotate to real position
                pos = rotate(pos, self.real_bearing-self.bearing)
                self.node.send(messaging.RelativePositionMessage('NECorner', 'AUV', messaging.CartesianPosition2D(*pos)))

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-f', '--pipeline_name', default = 'sim_sonar_walls', help="Name of pipeline file.")
    p.add_argument('-l', '--lines_name', default = 'wall_lines', help="Name of broadcast lines node.")
    p.add_argument('-s', '--scale_name', default = 'wall_lines_scale', help="Name of broadcast range node.")
    p.add_argument('-b', '--arena_bearing_correction', default=-90, help="Bearing of real north in fake coords.") # competition 17
    p.add_argument('-t', '--tolerance', default=5,
                   help="maximum difference between expected and actual angle of lines before rejecting lines")
    p.add_argument('-d', '--wall_length', default=50, help="Length of the back wall (m)")
    opts, args = p.parse_known_args()
    
    lm = LocationManager(opts)
    lm.run()