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
    Load_Pipeline = 'default'
    Pipeline_File = 'pipelines/follow_pipe.pipe'    
    Centre_Name = 'pipe',
    Lines_Name = 'pipe',
    #Timeouts
    Ready_Timeout = 200
    Lost_Timeout = 2
    Following_Timeout = 2
    #
    Prop_Speed = 40
    Target_Width = 0.2, #of image
    Width_Error  = 0.1, #of image
    Centre_Error = 0.1, #of image
    Align_Error  = 5, #degrees
    #bin = 11,
    threshold = 0.1,
    Strafe_p  = 255,
    depth_p   = 0.1,
    depth_enable = False

class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts) 
        self.node.join("processing")
        self.__pl = pipeline.Model(self.node, self.options.Load_Pipeline)
        # parameters to say if the auv is above the pipe
        self.centred = threading.Event()
        self.aligned = threading.Event()
        self.depthed = threading.Event()
        self.ready = threading.Event()

    def loadPipeline(self):
        self.__pl.load(Options.Pipeline_File)

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
        if self.options.Load_Pipeline is not None:
            saved_pipeline_state = self.__pl.get()
            self.loadPipeline()
        debug('Waiting for ready...')
        self.ready.wait(self.options.Ready_Timeout)
        while True:
            if not self.ready.is_set():
                error("Took too long to become ready, aborting")
                self.cleanup()
                break
            info("Above pipe, heading forward...")
            self.auv.prop(self.options.Prop_Speed)
            while self.ready.is_set():
                time.sleep(1)
            info("Re-aligning with pipe...")
            self.auv.prop(0)
            self.ready.wait(self.options.Lost_Timeout)
            #This seems to a mix of old and unused stuff
            # we are no longer aligned with the pipe, stop, and check whether we
            # are still attempting to align (ie enable is still true)
            #self.enable_lock.acquire()
            #if not self.enable:
            #    error('The pipe seems to have gone, waiting...')
            #    # if it isnt, give it lost_timeout to reenable
            #    self.enable_lock.wait(Options.Lost_Timeout)
            #if not self.enable:
            #    error("Lost the pipe, giving up")
            #    self.enable_lock.release()
            #    self.cleanup()
            #    break
            #self.enable_lock.release()
            #info('Repeating...')
            # we found the pipe again, loop around
        info('Finished')
        if self.options.Load_Pipeline is not None:
            self.__pl.set(saved_pipeline_state)

    def cleanup(self):
        error('does this ever get called?')
        self.node.removeObserver(self)


