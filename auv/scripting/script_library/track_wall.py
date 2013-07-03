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

class TrackWall(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.strafekP = 1000 #controls strafe speed, int [-127, 127]
            self.strafeLimit = 60
            self.strafeOffset = 10
            self.wallDistancekP = -1000
            self.depth = 2 #depth in metres
            self.useDepth = True
            self.maximumRunTime = 200 #run time in seconds
            self.forwardAngle = math.pi/4
            self.changeDifference = 0.01
            self.maximumBearingChange = 5
            self.targetDistance = 0.1
            self.initialLocation = (Simulation_Datum+NorthEastDepthCoord(-18, 5, 0)).toWGS84()
            self.initialBearing = 80
            self.linesName = 'track_wall'
            self.pipeline = 'track_wall2'
    
    class Debug(AI.Script.Debug):
        def __init__(self):
            self.angle1 = 0
            self.angle2 = 0
            self.distance1 = 0
            self.distance2 = 0
            
    def __init__(self):
        AI.Script.__init__(self)
        self.angle1 = 0
        self.angle2 = 0
        self.distance1 = 0
        self.distance2 = 0
        
    def onLinesMessage(self, m):
        if m.name != self.options.linesName:
            return
        """
                              \
                             / \
                            /a2/\
                distance2  /     \  Wall
                       /          \
        ________   /  b \        /a\             b = forwardAngle
        |_AUV___| -------\------/---\
                      distance       \
        
        Strategy:
        -Calculate distances/angles
        -If there is a wall to our left (indicated by distance2), turn to face this wall
        -Else align with the wall in front
        -Control distance using prop, target is targetDistance
        -Use distance2 to set strafe, so if distance2 > target distance strafe left (since there is no wall in the way)
            if distance2 < target distance strafe right (since there is a wall, which we will be turning to face, so to avoid crashing we need to reverse a bit)
        """
        try:
            #Here we try projecting the lines first to avoid oscilating on corners, where as we see the next wall,
            #we turn to face it, only to now see theprevious wall so turn to face that etc
            #With projection, this isnt an issue as the auv is on the line so the distance where this can happen is 0
            #
            #              _______
            #            /            _ projected (and interpolated
            #        /                     _
            #    /             _                _
            #/               /      _                _
            #            /               _                _
            #        /     original           _                _
            #    /                                 _                _
            angle, distance = self.calculate_intersection(m.lines, project = self.options.targetDistance)
        except InsufficientDataError:
            warning("Falling back to old method, risk of oscillation")
            #fall back to original method (occasionally happens when at an angle with only straight wall in front
            #as forward line can miss projection
            try:
                angle, distance = self.calculate_intersection(m.lines)
            except InsufficientDataError:
                return
        angled_line = True #sometimes we cant get the angled line, so keep a record of this
        #for the second line to the left, rotate the image first and then do the same calcualtion (rotation is mathematical,
        #so need -forwardAngle
        try:
            angle2, distance2 = self.calculate_intersection(m.lines, rotate_angle=-self.options.forwardAngle)
        except InsufficientDataError:
            angle2, distance2 = 0, 0
            angled_line = False
            
        #convert to degrees for bearing
        angle = math.degrees(angle)
        angle2 = math.degrees(angle2)
        
        #set debug values
        self.angle1 = angle
        self.angle2 = angle2
        self.distance1 = distance
        self.distance2 = distance2
        debug('Data is %f, %f, %f, %f' %(angle,distance,angle2,distance2))
        
        #Bearing Control
        #if we have we have the distance of the angled line, and the length of the line is notably less then the parrallel
        #component of the forward line, then we have a wall to our left, so turn and face it
        if angled_line and (distance-0.5)/math.cos(self.options.forwardAngle)-(distance2-0.5) > self.options.changeDifference:
            debug('Detected wall in direction of travel, turning to wall')
            #The maximum turn per message means that the control loops for strafe and prop have time to adjust
            next_angle = max(-self.options.maximumBearingChange, angle2-90-math.degrees(self.options.forwardAngle))
            self.auv.bearing(self.auv.current_bearing+next_angle)
        else:
            self.auv.bearing(self.auv.current_bearing+angle-90)
            
        #Prop Control
        speed = (0.5+self.options.targetDistance-distance)*self.options.wallDistancekP+self.options.strafeOffset
        debug('Setting speed %f' %(speed))
        self.auv.prop(int(speed))
        
        #Strafe Control
        if distance2:
            strafe_speed = (distance2-0.5-self.options.targetDistance)*self.options.strafekP
            strafe_speed = self.options.strafeLimit if strafe_speed > self.options.strafeLimit else strafe_speed
            strafe_speed = -self.options.strafeLimit if strafe_speed < -self.options.strafeLimit else strafe_speed
            self.auv.strafe(int(strafe_speed))
            debug('Setting strafe %f' %(strafe_speed))
        #if we occasionally end up stuck travelling right, uncommment the following:
        #else:
        #    self.auv.strafe(int(self.options.strafeLimit))
        #    debug('Setting strafe %f' %(self.options.strafeLimit))
        
    def calculate_intersection(self, lines, rotate_angle = 0, project = 0):
        """
        If rotate_angle is given, rotate all the lines first (rotate_angle in radians)
        If project is given project lines first
        If there are lines directly in front, we can work out angle/distance by these lines
        If there isnt, we need to draw a line between the lines either side and then use this new line to work out angle/distance
        Uncomment sending lines messages and use track_wall_debug to see where these lines are ending up
        """
        #remember image y coordinate is inverse to normal y,
        #normal y = 1-image y
        
        #Rotating
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
                new_line = msg.Line(msg.floatXY(centre_x, centre_y), (line.angle-rotate_angle)%math.pi, line.length, 0)
                lines2.append(new_line)
            lines = lines2
            
        #Projecting
        #project line: take c to c2 by (where l is the projection distance)
        #c2 = c - (lsin a) (normal coords) so image coords c2 = c + (-lsin a)
        #         (lcos a)                                          ( lcos a)
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
            
        #self.node.send(msg.LinesMessage('out1'+str(bool(rotate_angle)),lines))
        
        lines_in_front = []
        top_least = None # line not in front, above forward line, closest to forward line
        top_least_p_length = 0
        bottom_most = None # line not in front, below forward line, closest to forward line
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
            #if above center (remember y coord starts at 0 at top of image
            elif line.centre.y+p_length<0.5:
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
            #self.node.send(msg.LinesMessage('out2'+str(bool(rotate_angle)),lines_in_front))
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
            #self.node.send(msg.LinesMessage('out2'+str(bool(rotate_angle)),[msg.Line(msg.floatXY((top_point[0]+bottom_point[0])/2,(top_point[1]+bottom_point[1])/2),
            #                                                 angle,
            #                                                 math.sqrt((top_point[0]-bottom_point[0])**2+(top_point[1]-bottom_point[1])**2),
            #                                                 0)]))
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
            

Script = TrackWall

if __name__ == "__main__":
    TrackWall.entry()
