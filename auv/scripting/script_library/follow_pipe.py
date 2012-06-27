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
from math import degrees, cos, sin
import time

class scriptOptions(aiScriptOptions):
    #Pipeline details
    follow_pipeline_file  = 'follow_pipe'
    centre_name = 'pipe'
    lines_name = 'pipe'
    histogram_name = 'pipe'
    #Timeouts
    ready_timeout = 30
    lost_timeout = 10
    # Calibration
    pipe_end     = 0.2 # of image (range +0.5 to -0.5)
    target_width = 0.1 # of image
    width_error   = 0.1 # of image
    centre_error = 0.2 # of image
    align_error  = 5   # degrees 
    average_time = 1   # seconds
    intensity_trigger = 0.10
    y_disp_trigger = 0.3
    # Control
    prop_speed = 80
    strafe_kP  = -50
    
    #TODO: this doesn't actually do anythign at the moment
    # its not tuned
    depth_kP   = 0.1

    class Meta:
        dynamic = [
            'ready_timeout', 'lost_timeout',
            'strafe_kP', 'depth_kP',
            'prop_speed', 'average_time'
        ]


class script(aiScript):
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
        
        # controllers for staying above the pipe
        self.depthControl = PIDController((self.options.depth_kP,0,0))
        self.strafeControl = PIDController((self.options.strafe_kP,0,0))

        self.yAverage = TimeAverage(1)

    def onLinesMessage(self, m):
        if m.name != self.options.lines_name:
            debug('follow pipe: ignoring lines message %s != %s' % (m.name, self.options.lines_name))
            return
        if len(m.lines):
            #
            # Adjust the AUV's bearing to align with the pipe
            # calculate the average angle of the lines (assumes good line finding)
            angle = sum([x.angle for x in m.lines])/len(m.lines)
            
            # calculate the bearing of the pipe (relative to the sub),
            # mod 180 as we dont want to accidentally turn the sub around
            corrected_angle=90-degrees(angle)%180 #TODO this mod 180 probably is not needed from observation of cam following
            debug('follow pipe: mean lines direction: %g (uncorrected=%g)' % (corrected_angle, angle))
            
            # work out the actual bearing of the pipe using the current bearing of the AUV
            current_bearing = self.auv.getBearing()
            if current_bearing and self.enabled:
                self.auv.bearing((current_bearing-corrected_angle)%360) #- as angle is opposite direction to bearing
            
            if abs(corrected_angle) < self.options.align_error: self.aligned.set()
            else: self.aligned.clear()
            
            
            #
            # Adjust the depth of the AUV acording to the width of the pipe in the image 
            # we can only calculate width if we have 2 lines, and dont bother if the angle is too different
            if len(m.lines) == 2 and degrees(abs(m.lines[0].angle-m.lines[1].angle)) < 15:
                # dot product line between centeres with normal perpendicular to pipe
                #             |
                #             |    /   /
                #     \_      |   /   /        s = -corrected_angle
                #       \_    |s /   /
                #         \_  | /   /
                #           \_|/   /
                #_________s___|___/_____________
                #            /|  /
                #           / | /
                #             |
                #             |
                #             |
                #             |
                #             |
                width = abs(-cos(corrected_angle)*(m.lines[1].x-m.lines[0].x)-sin(corrected_angle)*(m.lines[1].x-m.lines[0].y)) 
                debug('follow pipe: estimated pipe width: %g' % width)
                
                # update the PID controller to get the required change in target depth
                width_error = width - self.options.target_width
                debug("Width error = %f" % (width_error))
                
                dive = self.depthControl.update(width_error)
                if self.auv.current_depth and self.enabled: #again, none depths
                    self.auv.depth(self.auv.current_depth + dive)
                
                if width_error < self.options.width_error: self.depthed.set()
                else: self.depthed.clear()
            
                
            # set the flags that show if we're above the pipe
            if self.centred.is_set() and self.aligned.is_set():# and self.depthed.is_set(): #TODO: fix this bit
                self.ready.set()
                debug('ready over pipeline')
                self.log('Pipeline follower managed to align itself over the pipe.')
            else:
                debug('centred:%s aligned:%s depthed:%s' %
                        (self.centred.is_set(), self.aligned.is_set(), self.depthed.is_set())
                )
                self.ready.clear()


    def onCentreMessage(self, m):
        if m.name != self.options.centre_name:
            debug('follow pipe: ignoring centre message %s != %s' % (m.name, self.options.centre_name))
            return
            
        debug(str((m.x,m.y)))
        
        
        info('Centre error: %f' %(m.x - 0.5))
        if abs(m.x) == 0:
            warning('centre at 0')
            self.centred.clear()
        strafe = int(self.strafeControl.update(m.x - 0.5))
        if strafe < -127:
            strafe = -127
        elif strafe > 127:
            strafe = 127
        if self.enabled:
            self.auv.strafe(strafe)
        info('pipe follow: strafing %i' %(strafe))
        
               
        # y average is used to find the end of the pipe
        yCoord = self.yAverage.update(m.y - 0.5)
        info("pipe y coord = %f" % (yCoord,))
        if(yCoord > self.options.pipe_end):
            self.pipeEnded.set()
        else: self.pipeEnded.clear()
        
        
        # set the flag used for determining if we're above the pipe
        debug('mx =  %g, self.options./centre_error = %g' % (float(m.x), self.options.centre_error))
        if abs(m.x -0.5) < abs(self.options.centre_error):
            self.centred.set() #ie within circle radius centre error
        else:
            self.centred.clear()
        

    def followPipeUntil(self, condition):
        debug('followPipeUntil: %s', condition.is_set())
        while not condition.is_set():
            self.auv.prop(0)
            # check we're still good to go, giving a little time to re-align the 
            # pipe if needed
            info("Re-aligning with pipe...")
            self.ready.wait(self.options.lost_timeout)
            if not self.ready.is_set():
                return False
            
            info("Above pipe, heading forward...")
            self.auv.prop(self.options.prop_speed)
            
            # go forward for a bit while we're still above the pipe
            while self.ready.is_set():
                time.sleep(0.2)
                if self.pipeEnded.is_set():
                    debug('Reached end of pipe, turning')
                    self.auv.prop(0)
                    self.enabled = False
                    self.auv.bearingAndWait((self.auv.current_bearing+180)%360)
                    self.enabled = True
        
        return True
  

    def run(self):
        # first we need to check we've actually found the pipe
        # the detector doesn't really look for the pipe it just looks
        # for yellow things in the downward camera, so there will be
        # a few false positives
        self.log('Attempting to align ove the pipe.')
        follow_pipe_file = self.options.follow_pipeline_file
        self.request_pl(follow_pipe_file)
        
        # now we wait for messages allowing us to work out how to align with
        # the pipe, but if this is taking too long then just give up as we've
        # probably drifted away from the pipe
        debug('Waiting for ready...')
        self.ready.wait(self.options.ready_timeout)
        if not self.ready.is_set():
            self.log('Pipeline follower could not position itself over the pipe (timed out).')
            error("Took too long to become ready, aborting")
            self.drop_pl(follow_pipe_file)
            return 'ABORT'
        
        for i in range(3):
            self.log('Attempting to follow pipe.')
            self.auv.prop(self.options.prop_speed)
            # follow the pipe along until the end
            if not self.followPipeUntil(self.pipeEnded):
                self.log('Lost the pipeline...')
                error("Pipeline lost on pass %d" %(i,))
                self.drop_pl(follow_pipe_file)
                return 'LOST'
            
            # turn 180
            self.log('Detected the end of the pipeline, turning and heading back.')
            info("Reached end of pass. Doing 180")
            self.auv.prop(0)
            self.auv.bearing((self.auv.getBearing()-180)%360)
        
        self.drop_pl(follow_pipe_file)
        self.log('Finished follwoing the pipe.')
        info('Finished pipe following')
        return 'SUCCESS'


