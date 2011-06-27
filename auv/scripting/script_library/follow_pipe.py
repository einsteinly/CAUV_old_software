#!/usr/bin/env python

import cauv
import cauv.messaging as messaging
import cauv.control as control
import cauv.pipeline as pipeline
import cauv.node

from cauv.debug import debug, info, warning, error
from AI_classes import aiScript, aiScriptOptions

import threading
from math import degrees, cos, sin
import time

class scriptOptions:
    #Pipeline details
    load_pipeline = 'default'
    pipeline_file = 'pipelines/follow_pipe.pipe'    
    centre_name = 'pipe',
    lines_name = 'pipe',
    #Timeouts
    ready_timeout = 200
    lost_timeout = 2
    # Calibration
    target_width = 0.2, #of image
    width_error  = 0.1, #of image
    centre_error = 0.1, #of image
    align_error  = 5,   #degrees
    # Control
    prop_speed = 40
    threshold = 0.1,
    Strafe_p  = 255,
    depth_p   = 0.1,
    depth_enable = False,




class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts) 
        self.node.join("processing")
        self.__pl = pipeline.Model(self.node, self.options.load_pipeline)
        # parameters to say if the auv is above the pipe
        self.centred = threading.Event()
        self.aligned = threading.Event()
        self.depthed = threading.Event()
        self.corners = threading.Event()
        self.ready = threading.Event()

    def loadPipeline(self):
        self.__pl.load(Options.pipeline_file)

    def onLinesMessage(self, m):
        if m.name != self.options.lines_name:
            debug('follow pipe: ignoring lines message %s != %s' % (m.name, self.options.lines_name))
            return
        #get current bearing asap, before we turn to much
        current_bearing = self.auv.getBearing()
        if len(m.lines):
            # calculate the average angle of the lines (assumes good line finding)
            angle = sum([x.angle for x in m.lines])/len(m.lines)
            # calculate the bearing of the pipe (relative to the sub), but mod 180 as we dont want to accidentally turn the sub around
            corrected_angle=90-degrees(angle)%180
            debug('follow pipe: mean lines direction: %g (uncorrected=%g)' % (corrected_angle, angle))
            if current_bearing: #watch out for none bearings
                self.auv.bearing((current_bearing-corrected_angle)%360) #- as angle is opposite direction to bearing
            #we can only calculate width if we have 2 lines, and dont bother if the angle is too different
            if len(m.lines) >= 2 and degrees(abs(m.lines[0].angle-m.lines[1].angle)) < 15 and self.depth_enable:
                xpos = []
                ypos = []
                for r in m.lines:
                    xpos.append(r.centre.x)
                    ypos.append(r.centre.y)
                # modulus of the cross product of the delta position of centres of 2 lines, and a unit vector of a line
                width = abs(sin(angle)*(max(xpos)-min(xpos))-cos(angle)*(max(ypos)-min(ypos))) 
                debug('follow pipe: estimated pipe width: %g' % width)
                if abs(width-self.options.target_width)>self.options.depth_error:
                    dive = (self.options.depth_p)*(width-self.options.target_width)
                    if self.auv.current_depth: #again, none depths
                        self.auv.depth(self.auv.current_depth+dive)
                else:
                    dive = 0
                info('Angle: %f, Pipe/Image ratio: %f, Turn: %f, Change in depth: %f' %(angle, width, corrected_angle, dive))
            else:
                dive = 0
                info('Angle: %f, Turn: %f, Not enough lines or lines to different for depth estimate or depth not enabled.' %(angle, corrected_angle))
            if abs(corrected_angle) < self.options.align_error: self.aligned.set()
            else: self.aligned.clear()
            if bool(dive): self.depthed.set()
            else: self.depthed.clear()
            if self.centred.is_set() and self.aligned.is_set() and self.depthed.is_set(): self.ready.set()
            else: self.ready.clear()

    def onCentreMessage(self, m):
        if m.name != self.options.centre_name:
            debug('follow pipe: ignoring centre message %s != %s' % (m.name, self.centre_name))
            return
        st = int((m.x-0.5)*self.strafe_p)
        self.auv.strafe(st)
        if m.x**2 + m.y**2<self.options.centre_error**2: self.centred.set() #ie within circle radius centre error
        else: self.centered.clear()
        info('pipe follow: strafing %i' %(st))

    def onCornersMessage(self, m):
        #TODO: process message
        info("Corners message")

    """
    def onHistogramMessage(self, m):
        #enable/disable based on histogram messages
        self.enable_lock.acquire()
        if (m.bins[self.bin] > self.threshold) ^ self.enable:
            self.enable = m.bins[self.bin] > self.threshold
            self.enable_lock.notify()
        self.enable_lock.release()
    """


    def followPipeUntil(self, condition):
        while not condition.is_set():
            # check we're still good to go, giving a little time to re-align the 
            # pipe if needed
            info("Re-aligning with pipe...")
            if not self.ready.wait(self.options.lost_timeout)
                return False
            
            info("Above pipe, heading forward...")
            self.auv.prop(self.options.prop_speed)
            
            # go forward for a bit while we're still above the pipe
            while self.ready.is_set():
                time.sleep(1)
        
        return True
  

    def run(self):
        # first we need to check we've actually found the pipe
        # the detector doesn't really look for the pipe it just looks
        # for yellow things in the downward camera, so there will be
        # a few false positives
        
        # TODO: pull in pipe_confirm
        # if not PipeConfirmer.isPipe() return;
        
        # by this point we think we've found the pipe
        # next step is to setup the pipelines we'll need for pipe following
        if self.options.load_pipeline is not None:
            saved_pipeline_state = self.__pl.get()
            self.loadPipeline()
            
        # now we wait for messages allowing us to work out how to align with
        # the pipe, but if this is taking too long then just give up as we've
        # probably drifted away from the pipe
        debug('Waiting for ready...')
        if not self.ready.wait(self.options.ready_timeout):
            error("Took too long to become ready, aborting")
            return #timeout
        
        
        for i in range(3):
            # follow the pipe along until the end (when we can see the corners)
            if not followPipeUntil(self.corners):
                error("Pipeline lost on pass ", i);
                self.cleanup()
                return
            
            # turn 180
            info("Reached end of first pass. Doing 180")
            self.auv.prop(0)
            self.auv.bearing((self.auv.getBearing()-180)%360)
        
        # save the pipeline in case it's been edited
        if self.options.load_pipeline is not None:
            self.__pl.set(saved_pipeline_state)
        
        info('Finished pipe following')
        notify_exit('SUCCESS')
        self.cleanup()

    def cleanup(self):
        self.node.removeObserver(self)


