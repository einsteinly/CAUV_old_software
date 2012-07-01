#!/usr/bin/env python2.7

import cauv
import cauv.messaging as msg
from cauv.debug import debug, info, warning, error 
from AI_classes import aiScript, aiScriptOptions
from utils.control import PIDController

import time
import math
import traceback
from collections import deque

class InsufficientDataError(ValueError):
    pass

class scriptOptions(aiScriptOptions):
    strafeSpeed = 60 #controls strafe speed, int [-127, 127]
    wallDistancekP = -500
    depth = 0 #depth in metres
    runTime = -300 #run time in seconds
    #doPropLimit = 20 #controls prop limit for dist adjustment, int [-127, 127]
    forward_angle = math.pi/6
    change_difference = 0.01
    target_distance = 0.1
    angles_to_avg = 5
    
    class Meta:
        # list of options that can be changed while the script is running
        dynamic = [
                'strafeSpeed',
                'wallDistancekP',
                'depth',
                #'doPropLimit'
                ]


class script(aiScript):
    def __init__(self, script_name, opts, state):
        aiScript.__init__(self, script_name, opts, state)
        self.node.subMessage(msg.LinesMessage())
        # self.node is set by aiProcess (base class of aiScript)
        # self.auv is also available, and can be used to control the vehicle
        # self.options is a copy of the option structure declared above
        self.angles = deque(maxlen=self.options.angles_to_avg)
        
    def onLinesMessage(self, m):
        try:
            angle, distance = self.calculate_intersection(m.lines, project = self.options.target_distance)
        except InsufficientDataError:
            try:
                angle, distance = self.calculate_intersection(m.lines)
            except InsufficientDataError:
                return
        angled_line = True
        try:
            angle2, distance2 = self.calculate_intersection(m.lines, rotate_angle=-self.options.forward_angle)
        except InsufficientDataError:
            angle2, distance2 = 0, 0
            angled_line = False
        angle = math.degrees(angle)
        angle2 = math.degrees(angle2)
        debug('Data is %f, %f, %f, %f' %(angle,distance,angle2,distance2))
        if angled_line and distance-distance2 > self.options.change_difference:
            debug('Detected wall in direction of travel, turning to wall')
            self.auv.bearing(self.auv.current_bearing+angle2-90-degrees(self.options.forward_angle))
        else:
            self.auv.bearing(self.auv.current_bearing+angle-90)
        speed = (0.5+self.options.target_distance-distance)*self.options.wallDistancekP
        debug('Setting speed %f' %speed)
        self.auv.prop(int(speed))
        
    def calculate_intersection(self, lines, rotate_angle = 0, project = 0):
        """
        if rotate_angle is given, rotate all the lines first (rotate_angle in radians)
        also project lines to form "ideal path"
        If there are lines directly in front, we can work out angle/distance by these lines
        If there isnt, we need to draw a line between the lines either side and then use this new line to work out angle/distance
        """
        #remember image y coordinate is inverse to normal y,
        #normal y = 1-image y
        #line_centre (normal y) is rotation matrix*(line_centre-image_centre)+image_centre, angle ->angle-rotate
        #(cos a -sin a) ( x -0.5) + (0.5)
        #(sin a cos a ) (1-y-0.5)   (0.5)
        #so: centre_x = cos a*(x-0.5)-sin a*(1-y-0.5)+0.5
        #normal centre_y = sin a*(x-0.5)+cos a*(1-y-0.5)+0.5
        #centre_y = 1-(sin a*(x-0.5)+cos a*(1-y-0.5)+0.5)
        if rotate_angle:
            ca = math.cos(rotate_angle)
            sa = math.sin(rotate_angle)
            lines2 = []
            for line in lines:
                centre_x = ca*(line.centre.x-0.5)-sa*(0.5-line.centre.y)+0.5
                centre_y = 0.5-sa*(line.centre.x-0.5)-ca*(0.5-line.centre.y)
                new_line = msg.Line(msg.floatXY(centre_x, centre_y), line.angle-rotate_angle, line.length, 0)
                lines2.append(new_line)
            lines = lines2
        #project line
        #-(lsin a)
        # (lcos a)
        #            \
        #             \
        #     \       a\
        #      \      __\c
        #       \        \
        #        \        \
        #      c2 \        \
        #          \        \
        #           \
        #            \
        if project:
            lines2 = []
            for line in lines:
                centre_x = line.centre.x-project*math.sin(line.angle)
                centre_y = line.centre.y+project*math.cos(line.angle)
                new_line = msg.Line(msg.floatXY(centre_x, centre_y), line.angle-rotate_angle, line.length, 0)
                lines2.append(new_line)
            lines = lines2
        lines_in_front = []
        top_least = None
        top_least_p_length = 0
        bottom_most = None
        bottom_most_p_length = 0
        #   /        ___                                            y=0
        #  /   ____|    \     |                                     |
        # /__|___-dB_____\pi/2-a                                    v
        # \               \   | l/2*cos(pi/2-a)=l/2*sin(a)
        #   \              \  |
        #     \             \ |
        #       \            \|c
        #         \        a  \  
        #           \     _____\____
        for line in lines:
            p_length = math.sin(line.angle%math.pi)*line.length/2 #perpendicular length from centre to furthermost point
            if abs(line.centre.y-0.5)<=p_length:
                lines_in_front.append(line)
            #if above center
            if line.centre.y+p_length<0.5:
                #if closer to center than any previous lines
                if top_least:
                    if line.centre.y+p_length > top_least.centre.y+top_least_p_length:
                        top_least = line
                        top_least_p_length = p_length
                else:
                    top_least = line
                    top_least_p_length = p_length
            #elif below center
            elif line.centre.y-p_length>0.5:
                #if closer to center than any previous lines
                if bottom_most:
                    if line.centre.y-p_length < bottom_most.centre.y-bottom_most_p_length:
                        bottom_most = line
                        bottom_most_p_length = p_length
                else:
                    bottom_most = line
                    bottom_most_p_length = p_length
        if lines_in_front:
            debug('Taking average of lines directly in front.')
            angle = 0
            distance = 0
            for line in lines_in_front:
                angle += line.angle
                distance += line.centre.x-math.tan(math.pi/2-line.angle)*(line.centre.y-0.5)
            angle /= len(lines_in_front)
            distance /= len(lines_in_front)
            #  \ 
            #   \
            #  a \
            #_____\
            #     |\
            #     |_\c
            #  c.x-d \
        else:
            #check lines exist
            if not top_least or not bottom_most:
                debug('Insufficient information')
                raise InsufficientDataError
            debug('Joining lines to estimate angle.')
            # \
            #  \
            #   \
            #    \
            #     \
            #    a \
            #_______\
            top_point = (top_least.centre.x+top_least.length/2*math.cos(top_least.angle),
                         top_least.centre.y+top_least.length/2*math.sin(top_least.angle))
            bottom_point = (bottom_most.centre.x-bottom_most.length/2*math.cos(bottom_most.angle),
                            bottom_most.centre.y-bottom_most.length/2*math.sin(bottom_most.angle))
            angle = math.atan2(bottom_point[1]-top_point[1], bottom_point[0]-top_point[0])
            # 
            #    \
            #______\
            #   d    \
            #          \
            #            \
            # distance is x coord weighted by y coord
            distance = (bottom_point[0]*bottom_point[1] + top_point[0]*top_point[1])/(bottom_point[1]+top_point[1])
        if project:
            distance += project
        return angle, distance
        
        
    def run(self):
        while True:
            #self.auv.strafe(self.options.strafeSpeed)
            time.sleep(10)
