#!/usr/bin/env python

import cauv
import cauv.messaging as messaging
import cauv.control as control
import cauv.pipeline as pipeline
import cauv.node

from cauv.debug import debug, info, warning, error
from AI_classes import aiScript

import threading
from math import degrees, cos, sin
import time

class Options:
    Node_Name = 'py-pf'
    Load_Pipeline = 'default'
    Pipeline_File = 'pipelines/follow_pipe.pipe'    
    Ready_Timeout = 200
    Lost_Timeout = 2
    Following_Timeout = 2
    Prop_Speed = 40

class script(aiScript):
    def __init__(self,
                 centre_name = 'pipe',
                 lines_name = 'pipe',
                 target_width = 0.2,
                 width_error  = 0.1,
                 centre_error = 0.1,
                 align_error  = 5,
                 bin = 11,
                 threshold = 0.1,
                 strafe_p  = 255,
                 depth_p   = 0.1,
                 depth_enable = False):
        aiScript.__init__(self, Options.Node_Name) 
        self.node.join("processing")
        self.__pl = pipeline.Model(self.node, Options.Load_Pipeline)
        # parameters to say if the auv is above the pipe
        self.alignment_lock = threading.Lock()
        self.centre_name = centre_name
        self.lines_name = lines_name
        self.ready_c = threading.Condition(self.alignment_lock) #condition to notify when ready state changes
        self.centred = False
        self.aligned = False
        self.depthed = False
        self.ready = False
        # proportions for strafing and diving (relative to pipe position and width respectively)
        self.strafe_p = strafe_p
        self.depth_p = depth_p
        # target width of pipe as a proportion the image
        self.target = target_width
        # acceptable limits to say we are over the pipe
        self.align_error = align_error
        self.depth_error = width_error
        self.centre_error_2 = centre_error**2 #saves squaring it later
        # enable allows centering and alignment of the sub
        self.enable_lock = threading.Condition(threading.Lock())
        self.enable = True
        # bin and min value of the bin from histogram node (used to decide whether to set enable=True)
        self.bin = bin
        self.threshold = threshold
        self.depth_enable = depth_enable

    def loadPipeline(self):
        self.__pl.load(Options.Pipeline_File)

    def onLinesMessage(self, m):
        if m.name != self.lines_name:
            debug('follow pipe: ignoring lines message %s != %s' % (m.name, self.lines_name))
            return
        #get current bearing asap, before we turn to much
        current_bearing = self.auv.getBearing()
        #check whether this is enabled
        #self.enable_lock.acquire()
        if len(m.lines):
            #self.enable_lock.release()
            # calculate the average angle of the lines (assumes good line finding)
            angle = sum([x.angle for x in m.lines])/len(m.lines)
            # calculate the bearing of the pipe (relative to the sub), but mod 180 as we dont want to accidentally turn the sub around
            corrected_angle=90-degrees(angle)%180
            # print "angle %f corrected angle %f" %(angle, corrected_angle)
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
                if abs(width-self.target)>self.depth_error:
                    dive = (self.depth_p)*(width-self.target)
                    if self.auv.current_depth: #again, none depths
                        self.auv.depth(self.auv.current_depth+dive)
                else:
                    dive = 0
                info('Angle: %f, Pipe/Image ratio: %f, Turn: %f, Change in depth: %f' %(angle, width, corrected_angle, dive))
            else:
                dive=0
                info('Angle: %f, Turn: %f, Not enough lines or lines to different for depth estimate or depth not enabled.' %(angle, corrected_angle))
            #acquire lock over alignment statuses
            self.ready_c.acquire()
            #set aligned +depthed status
            self.aligned = abs(corrected_angle) < self.align_error
            self.depthed = bool(dive)
            debug('aligned %i, depthed %i' % (self.aligned, self.depthed))
            if (self.centred and self.aligned and self.depthed) != self.ready:
                self.ready = self.centred and self.aligned and self.depthed
                #notify anything thats listening that the state has changed
                self.ready_c.notify()
            self.ready_c.release()
        #else: #dont forget to release the lock...
        #    self.enable_lock.release()

    def onCentreMessage(self, m):
        if m.name != self.centre_name:
            debug('follow pipe: ignoring centre message %s != %s' % (m.name, self.centre_name))
            return
        #check if enabled
        #self.enable_lock.acquire()
        #if self.enable:
        #self.enable_lock.release()
        st = int((m.x-0.5)*self.strafe_p)
        self.auv.strafe(st)
        self.alignment_lock.acquire()
        self.centred = m.x**2 + m.y**2<self.centre_error_2 #ie within circle radius centre error
        debug('pipe follow: centered = ' + str(self.centred))
        info('pipe follow: strafing %i' %(st))
        self.alignment_lock.release()
        #else: #dont forget to release lock, and make sure sub isnt still strafing when disabled
        #    self.auv.strafe(0)
        #    self.enable_lock.release()

    """
    def onHistogramMessage(self, m):
        #enable/disable based on histogram messages
        self.enable_lock.acquire()
        if (m.bins[self.bin] > self.threshold) ^ self.enable:
            self.enable = m.bins[self.bin] > self.threshold
            self.enable_lock.notify()
        self.enable_lock.release()
    """

    #def pipe_follow(auv_node, au, **kwargs):
    def run(self):
        if Options.Load_Pipeline is not None:
            saved_pipeline_state = self.__pl.get()
            self.loadPipeline()
        while True:
            debug('Waiting for ready...')
            self.ready_c.acquire()
            if not self.ready:
                # check if we are aligned, if not then wait until we are or timeout
                self.ready_c.wait(Options.Ready_Timeout)
            if not self.ready:
                error("Took too long to become ready, aborting")
                self.ready_c.release()
                self.cleanup()
                break
            self.ready_c.release()
            info("Above pipe, heading forward...")
            self.auv.prop(Options.Prop_Speed)
            self.ready_c.acquire()
            if self.ready:
                # check we are still aligned, if so wait until we are no longer aligned
                self.ready_c.wait(Options.Following_Timeout)
            self.ready_c.release()
            info("Re-aligning with pipe...")
            self.auv.prop(0)
            # we are no longer aligned with the pipe, stop, and check whether we
            # are still attempting to align (ie enable is still true)
            self.enable_lock.acquire()
            if not self.enable:
                error('The pipe seems to have gone, waiting...')
                # if it isnt, give it lost_timeout to reenable
                self.enable_lock.wait(Options.Lost_Timeout)
            if not self.enable:
                error("Lost the pipe, giving up")
                self.enable_lock.release()
                self.cleanup()
                break
            self.enable_lock.release()
            info('Repeating...')
            # we found the pipe again, loop around
        info('Finished')
        if Options.Load_Pipeline is not None:
            self.__pl.set(saved_pipeline_state)

    def cleanup(self):
        error('does this ever get called?')
        self.node.removeObserver(self)


