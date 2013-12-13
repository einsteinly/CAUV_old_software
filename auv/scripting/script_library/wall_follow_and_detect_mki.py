#!/usr/bin/env python2.7

import AI
from cauv.debug import debug, warning, error, info
from utils.boundedtypes import MotorValue

import cauv.messaging as msg

import time
import math
import traceback
import threading
from collections import deque
from utils import event

class WallFollow(event.EventLoop, AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            #For the wall tracking:
            self.useDepth = True
            self.depth = 1.3
            self.forward_speed = AI.OptionWithMeta(50, opt_type=MotorValue)
            self.initialBearing = 280
            self.track_distance = 0.007 # PLACEHOLDER VALUE
            self.linesName = 'wall_lines'
            #For the buoy detection:
            self.Sightings_Period = 1.0
            self.Required_Sightings = 5
            self.Max_Horizontal_Variation = 0.7
            self.Max_Vertical_Variation = 0.4
            self.Max_Samples = 7
            self.Max_Sample_Time = 10.0
            self.Pipeline_Name = "detect_buoy_sim"
            self.Circles_Name = "buoy"
 
    class Debug(AI.Detector.Debug):
        def __init__(self):
            self.value = False

    def __init__(self):
        event.EventLoop.__init__(self)
        AI.Script.__init__(self)
        self.load_pipeline(self.options.Pipeline_Name)
        self.node.subMessage(msg.CirclesMessage())
        self.node.addObserver(self)
        self.log('Looking for the buoy')
        self.circles = deque(maxlen = self.options.Max_Samples)
        self.times = deque(maxlen = self.options.Max_Samples)
        self.foundBuoy = False
        if self.options.useDepth:
            self.auv.depth(self.options.depth)
        self.auv.bearingAndWait(self.options.initialBearing)
        print('Prepping to go to subMessage')
        self.node.subMessage(msg.LinesMessage())
        print('Past submessage')
        self.found = False
 
    @event.event_func                           #Puts function into a que
    def onCirclesMessage(self, m):
        #if m.name != self.options.Circles_Name:#If the data being received isn't the circle data
        #   return
        if len(m.circles) != 1:
            debug("Incorrect number of circles", 5)
            return
        debug("Saw circle", 5)                  #Note that one/more cirles seen
        self.circles.append(m.circles[0])       #Add a the data in circles[0] to the right of the circles deque
        self.times.append(time.time())          #Ditto with the clock time into the time deque
 
 
 
    @event.event_func  
    def onLinesMessage(self, m):
        speed = 0
        strafespeed = 0
        if self.found != True:
            print('Got to onLinesMessg')
            #print m
            
            distance, angle = self.Find_Lines(m.lines)
            angle = math.degrees(angle) 
            #Convert the angle into degrees (from radians)
            
            print 'The angle is ', angle
            print 'The distance is ', distance
            
            #Check how far 'Cuda is from wall
            speed = 0
            speed = 1000*(distance - self.options.track_distance)
            if speed > 50:
                speed = 50
            if 0 < speed < 5:
                speed = 5
            if 0.003 < distance < 0.007:
                speed = 0
            
            print 'The speed is now', speed
            #The speed of 'Cuda is proportional to the distance from the wall
            self.auv.prop(int(speed))
                
            bearing = self.auv.bearing
            if 0.003 < distance < 0.01:
                changeinbear = (angle - 90)/2	
                self.auv.bearing(self.auv.current_bearing+changeinbear)
            
            #Strafe if parallel to close wall
            
            if distance < 0.007:
                if self.foundBuoy is False:
                    strafespeed = 0
                    strafespeed = 3000*(distance - self.options.track_distance)
                    if strafespeed > 15:
                        strafespeed = 15
                elif self.foundBuoy is True:
                    strafespeed = -(3000*(distance - self.options.track_distance))
                    time.sleep(1)
                    self.auv.strafe(int(strafespeed))
                    strafespeed = 0
                    self.found = True
                self.auv.strafe(int(strafespeed))
                print 'Strafe speed', strafespeed
            
            return

    @event.repeat_event(0.2, True)              #Puts function onto que every .2 seconds.
    def process(self):
        #Now that you are close to the wall, look for a buoy.
        now_t = time.time()                     #Make now_t = to the clock time.
        no_samples = len(self.times)            #Make no_samples = the length of the time data 
        #Find first sample in time frame
        #To get enough samples, know must be atleast 1 in the first no_samples-max_samples+1 of samples
        for sample_i in range(no_samples-self.options.Required_Sightings+1): #The range generates a sequence of numbers of size (length of time - the number of sightings required + 1)
            sample_t = now_t-self.times[sample_i]#sample_t = the clock time - the tested sample in the whole set of times
            if sample_t < self.options.Max_Sample_Time:#if sample_t < the maximum amount of time allowed
                start_i = sample_i
                break                           #Skip the subsequent else clause
        else:
            debug("Not enough circles in time.", 5)
            print 'Not enough circles'
            self.value = False
            return False                        #Some shit that exits when not enough circles are seen
        #Order samples by horizontal position
        sorted_samples = sorted(list(self.circles)[start_i:], key=lambda circ: circ.centre.x)
        #for each selection in the correct horizontal variation, repeat for vertical position
        for substart_i in range(len(sorted_samples)-self.options.Required_Sightings):
            for subend_i in range(len(sorted_samples)-1, substart_i+self.options.Required_Sightings-1, -1):
                debug("Horizontal {}".format(sorted_samples[subend_i].centre.x-sorted_samples[substart_i].centre.x), 5)
                if sorted_samples[subend_i].centre.x-sorted_samples[substart_i].centre.x>self.options.Max_Horizontal_Variation:
                    #samples too far apart, move back 1
                    continue
                #found largest subset starting at substart_i
                sorted_samples_vert = sorted(sorted_samples[substart_i:subend_i], key=lambda circ: circ.centre.y)  #from substart_i to subend_i in the list of samples sorted by horizontal position, create a new list called 'sorted_samples_vert' that sorts the items by vertical position.
                for substart_vert_i in range(len(sorted_samples_vert)-self.options.Required_Sightings):
                    debug("Vertical {}".format(sorted_samples_vert[substart_vert_i+self.options.Required_Sightings].centre.y-sorted_samples_vert[substart_vert_i].centre.y), 5)
                    if sorted_samples_vert[substart_vert_i+self.options.Required_Sightings].centre.y-sorted_samples_vert[substart_vert_i].centre.y<self.options.Max_Vertical_Variation: #if the items that are within horizontal variation are also within vertical variation, then return true.
                        print 'Buoy detected'
                        self.value = True
                        self.foundBuoy = True
                        return
                break
        print 'Nothing found'
        self.value = False
        self.foundBuoy = False
        return #If all else fails, return false.
        
        
    """                    __
                        A  AB = A line that represents a wall
                       /
    Centre of trace   /
       (0.5,0.5)     /
          SUB       /
           |       /
       s.y |______/           _________
            s.x  / CENTRE     SUBCENTRE (invs) = s 
                /  (of AB)        
               /___
              /    \
             / angle\
            /_ _ _ _|_ _
            B
    """
    


    def Find_Lines(self, lines):
        length = 0              #Length of line
        angle = 0               #Angle of line
        distance = 0            #Distance to line
        linecount = 0           #Counter for lines assessed
        for line in lines:      #looks in the data list 'lines' for each object called 'line' and does the suite for each in ascending order.
            t = msg.floatXY(math.cos(line.angle), math.sin(line.angle)) #Vector in the direction AB
            s = msg.floatXY(line.centre.x - 0.5, line.centre.y - 0.5) #Vector s
            distance += abs(((s.x)**2 + (s.y)**2)**.5)
            angle += line.angle
            length += line.length
            linecount += 1
        if linecount == 0:
            linecount = 1
        return distance/linecount, angle/linecount


Script = WallFollow

if __name__ == "__main__":
    WallFollow.entry()
