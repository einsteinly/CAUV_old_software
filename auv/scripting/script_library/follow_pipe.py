#!/usr/bin/env python2.7

import cauv
import cauv.messaging as messaging
import cauv.pipeline as pipeline
import cauv.node

from utils.control import PIDController
from utils.timeaverage import TimeAverage
from cauv.debug import debug, info, warning, error
from AI_classes import aiScript, aiScriptOptions

import threading
from math import degrees, cos, sin, pi
import time

class scriptOptions(aiScriptOptions):
    #Pipeline details
    follow_pipeline_file  = 'follow_pipe2'
    ellipses_name = 'pipe'
    lines_name = 'pipe'
    turns = 3
    #Timeouts
    ready_timeout = 30
    lost_timeout = 15
    # Calibration
    pipe_end     = 0.1 # of image (range +0.5 to -0.5)
    target_width = 0.05 # of image
    width_error  = 0.1 # of image
    centre_error = 0.2 # of image
    align_error  = 10   # degrees 
    average_time = 1   # seconds
    angle_consistency = 0.4
    # Control
    prop_speed = 80
    strafe_kP  = -50 # if we are to the left of the pipe then the pipe is on the right, so has positive center error
    #we want to move right, ie negative strafe, so set kP negative
    depth_kP = -1 # if we are too high the width is too small so the error appears negative
    #we want to dive, ie positive value to add to depth, so kP is negative

    class Meta:
        dynamic = [
            'ready_timeout', 'lost_timeout',
            'prop_speed', 'average_time',
        ]


class script(aiScript):
    debug_values = ['centre_error', 'angle_error', 'depth_error', 'angle', 'centred', 'depthed', 'aligned', 'pipeEnded', 'yVal']
    def __init__(self, script_name, opts, state):
        aiScript.__init__(self, script_name, opts, state) 
        self.node.join("processing")
        
        # parameters to say if the auv is above the pipe
        self.centred = threading.Event()
        self.aligned = threading.Event()
        self.depthed = threading.Event()
        self.pipeEnded = threading.Event()
        self.ready = threading.Event()
        self.enabled = True
        
        #debug values
        self.centre_error = 0
        self.angle_error = 0
        self.depth_error = 0
        self.angle = 0
        self.yVal = 0
        
        # controllers for staying above the pipe
        # proportional: if the value we give is positive, then the values we get is positive*sign(kP)
        self.depthControl = PIDController((self.options.depth_kP,0,0))
        self.strafeControl = PIDController((self.options.strafe_kP,0,0))

        self.yAverage = TimeAverage(1)

    def onLinesMessage(self, m):
        if m.name != self.options.lines_name:
            return
        if len(m.lines):
            #
            #          /
            #  angle  /
            #________/line
            #       /
            #
            # Adjust the AUV's bearing to align with the pipe
            # calculate the average angle of the lines (assumes good line finding)
            self.angle = sum([x.angle for x in m.lines])/len(m.lines)
            
            # calculate the bearing of the pipe (relative to the sub),
            # mod 180 as we dont want to accidentally turn the sub around
            corrected_angle=90-degrees(self.angle)%180 #TODO this mod 180 probably is not needed from observation of cam following
            
            #debug
            debug('Average lines direction: %g (uncorrected=%g)' % (corrected_angle, self.angle))
            self.angle_error = corrected_angle
            
            # work out the actual bearing of the pipe using the current bearing of the AUV
            if self.auv.current_bearing and self.enabled:
                self.auv.bearing((self.auv.current_bearing-corrected_angle)%360) #- as angle is opposite direction to bearing
            
            if abs(corrected_angle) < self.options.align_error:
                self.aligned.set()
            else:
                self.aligned.clear()
                            
            # set the flags that show if we're above the pipe
            debug('Centred:%s Aligned:%s Depthed:%s' %
                    (self.centred.is_set(), self.aligned.is_set(), self.depthed.is_set())
            )
            if self.centred.is_set() and self.aligned.is_set() and self.depthed.is_set():
                self.ready.set()
                self.log('Pipeline follower managed to align itself over the pipe.')
            else:
                self.ready.clear()

    def onEllipsesMessage(self, ms):
        if ms.name != self.options.ellipses_name:
            return
        try:
            m = ms.ellipses[0]
        except IndexError:
            warning('Incorrect ellipses message received')
        #centering
        debug('Centre error: %f' %(m.centre.x - 0.5))
        if abs(m.centre.x) == 0: #probably an error
            warning('Centre at 0, ignoring')
            self.centred.clear()
            return
        
        strafe = int(self.strafeControl.update(m.centre.x - 0.5))
        if strafe < -127:
            strafe = -127
        elif strafe > 127:
            strafe = 127
        if self.enabled:
            self.auv.strafe(strafe)
        info('Strafing %i' %(strafe))
        
        #debug info
        self.centre_error = m.centre.x - 0.5
               
        # y average is used to find the end of the pipe
        yCoord = self.yAverage.update(m.centre.y - 0.5)
        info("Averaged centre y location coord = %f" % (yCoord,))
        #if y is below a certain point and we are above the pipe
        if(yCoord > self.options.pipe_end) and self.ready.is_set():
            self.pipeEnded.set()
        else: self.pipeEnded.clear()
        
        #debug
        self.yVal = yCoord
        
        # set the flag used for determining if we're above the pipe
        if abs(m.centre.x -0.5) < abs(self.options.centre_error):
            self.centred.set() #ie within circle radius centre error
        else:
            self.centred.clear()
            
        #depthing
        if abs(self.angle%pi-m.angle%pi)>self.options.angle_consistency and \
        abs(self.angle%pi-m.angle%pi)<pi-self.options.angle_consistency:
            debug('Too much variation in angle between lines and ellipse (angles %f, %f respectively)' %(self.angle, m.angle))
            return
        depth = self.depthControl.update(m.minorRadius-self.options.target_width)
        debug(str(m.minorRadius))
        self.depth_error = depth
        debug('Width error: %f, diving: %f' %(m.minorRadius-self.options.target_width, depth))
        self.auv.depth(self.auv.current_depth+depth)
        if abs(self.options.target_width-m.minorRadius) < self.options.width_error:
            self.depthed.set()
        else:
            self.depthed.clear()

    def followPipeUntil(self, condition):
        debug('followPipeUntil: %s', condition.is_set())
        while True:
            self.auv.prop(0)
            # check we're still good to go, giving a little time to re-align the 
            # pipe if needed
            debug("Re-aligning with pipe...")
            if not self.ready.wait(self.options.lost_timeout):
                return False
            
            debug("Above pipe, heading forward...")
            self.auv.prop(self.options.prop_speed)
            
            # go forward for a bit while we're still above the pipe
            while self.ready.is_set():
                time.sleep(0.2)
                if condition.is_set():
                    return True
        
  

    def run(self):
        # first we need to check we've actually found the pipe
        # the detector doesn't really look for the pipe it just looks
        # for yellow things in the downward camera, so there will be
        # a few false positives
        self.log('Attempting to align over the pipe.')
        follow_pipe_file = self.options.follow_pipeline_file
        self.request_pl(follow_pipe_file)
        
        # now we wait for messages allowing us to work out how to align with
        # the pipe, but if this is taking too long then just give up as we've
        # probably drifted away from the pipe
        debug('Waiting for ready...')
        if not self.ready.wait(self.options.ready_timeout):
            self.log('Pipeline follower could not position itself over the pipe (timed out).')
            error("Took too long to become ready, aborting")
            self.drop_pl(follow_pipe_file)
            return 'ABORT'
        
        for i in range(self.options.turns):
            self.log('Attempting to follow pipe.')
            self.auv.prop(self.options.prop_speed)
            # follow the pipe along until the end
            if not self.followPipeUntil(self.pipeEnded):
                self.log('Lost the pipeline...')
                error("Pipeline lost on pass %d" %(i,))
                self.drop_pl(follow_pipe_file)
                return 'LOST'
                
            debug('Reached end of pipe, turning (turn %i)')
            self.auv.prop(0)
            self.enabled = False
            self.auv.bearingAndWait((self.auv.current_bearing+180)%360)
            self.enabled = True
        
        self.drop_pl(follow_pipe_file)
        self.log('Finished following the pipe.')
        info('Finished pipe following')
        return 'SUCCESS'


