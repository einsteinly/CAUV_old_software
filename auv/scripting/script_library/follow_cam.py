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
    follow_pipeline_file  = 'follow_cam.pipe'
    centre_name = 'cam'
    lines_name = 'cam'
    histogram_name = 'cam'
    #Timeouts
    ready_timeout = 30
    lost_timeout = 10
    # Calibration
    target_width = 0.2 # of image
    width_error   = 0.1 # of image
    centre_error = 0.2 # of image
    align_error  = 5   # degrees 
    average_time = 1   # seconds
    intensity_trigger = 0.10
    # Control
    prop_speed = 80
    strafe_kPID  = (-300, 0, 0)
    
    class Meta:
        dynamic = [
            'ready_timeout', 'lost_timeout',
            'strafe_kPID', 'depth_kPID',
            'prop_speed', 'average_time'
        ]


class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts) 
        self.node.join("processing")
        
        # parameters to say if the auv is in the middle and aligned to the pipe
        self.centred = threading.Event()
        self.aligned = threading.Event()
        self.ready = threading.Event()
        
        # controllers for staying above the pipe
        self.strafeControl = PIDController(self.options.strafe_kPID)

        self.yAverage = TimeAverage(1)

    def onLinesMessage(self, m):
        if m.name != self.options.lines_name:
            debug('follow pipe: ignoring lines message %s != %s' % (m.name, self.options.lines_name))
            return
        if len(m.lines):
            #
            # Adjust the AUV's bearing to align with River Cam by line detection of the bank
            # calculate the average angle of the lines (assumes good line finding)
            angle = sum([x.angle for x in m.lines])/len(m.lines)
            
            # calculate the bearing of River Cam (relative to the sub),
            # mod 180 as we dont want to accidentally turn the sub around
            corrected_angle=90-degrees(angle)%180
            debug('follow pipe: mean lines direction: %g (uncorrected=%g)' % (corrected_angle, angle))
            


            if abs(corrected_angle) < self.options.align_error: 
                self.aligned.set()

            #Align the AUV to River Cam if the error is outside of tolerance
            else: 
		current_bearing = self.auv.getBearing()
		if current_bearing: #watch out for none bearings
		self.auv.bearing((current_bearing-corrected_angle)%360) #- as angle is opposite direction to bearing
		self.aligned.clear()

            # set the flags that show if we're above the pipe
            if self.centred.is_set() and self.aligned.is_set():# and self.depthed.is_set(): #TODO: fix this bit
                self.ready.set()
                debug('ready to follow Cam')
                self.log('River Cam follower managed to align itself over the Cam.')
            else:
                debug('centred:%s aligned:%s' %
                        (self.centred.is_set(), self.aligned.is_set())
                )
                self.ready.clear()
            
            
    def onCentreMessage(self, m):
        if m.name != self.options.centre_name:
            debug('follow Cam: ignoring centre message %s != %s' % (m.name, self.options.centre_name))
            return
        
        
        info('Centre error: %f' %(m.x - 0.5))
        if abs(m.x) < 0.01:
            warning('ignoring centre at 0')
            return
        strafe = int(self.strafeControl.update(m.x - 0.5))
        if strafe < -127:
            strafe = -127
        elif strafe > 127:
            strafe = 127
        self.auv.strafe(strafe)
        info('Cam follow: strafing %i' %(strafe))


        # set the flag used for determining if we're at the centre of River Cam
        debug('mx =  %g, self.options.centre_error = %g' % (float(m.x), self.options.centre_error))
        if m.x**2 < self.options.centre_error**2:
            self.centred.set() #ie within circle radius centre error
        else:
            self.centred.clear()


    def run(self):
        self.log('Initiating following river cam.')
        follow_cam_file = self.options.follow_cam_file
        self.request_pl(follow_cam_file)
        
        # now we wait for messages allowing us to work out how to align with
        # the pipe, but if this is taking too long then just give up as we've
        # probably drifted away from the pipe
        debug('Waiting for ready...')
        self.ready.wait(self.options.ready_timeout)
        if not self.ready.is_set():
            self.log('Cam follower could not position itself at the centre of River Cam (timed out).')
            error("Took too long to become ready, aborting")
            self.drop_pl(follow_cam_file)
            self.notify_exit('ABORT')            
            return #timeout
        

	self.auv.prop(self.options.prop_speed)
	# follow the pipe along until the end
	if not self.followPipeUntil(self.pipeEnded):
	    self.log('Lost River Cam...')
	    error("River Cam is lost")
	    self.drop_pl(follow_cam_file)
	    self.notify_exit('LOST')
	    return
            

        
        self.drop_pl(follow_cam_file)
        self.log('Finished follwoing River Cam.')
        info('Finished River Cam following')
        self.notify_exit('SUCCESS')

