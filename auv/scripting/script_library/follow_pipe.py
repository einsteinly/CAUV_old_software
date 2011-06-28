#!/usr/bin/env python

import cauv
import cauv.messaging as messaging
import cauv.pipeline as pipeline
import cauv.node

from utils.control import PIDController
from cauv.debug import debug, info, warning, error
from AI_classes import aiScript, aiScriptOptions

import threading
from math import degrees, cos, sin
import time


class scriptOptions(aiScriptOptions):
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
    centre_error = 0.1, #of image
    align_error  = 5,   #degrees
    # Control
    prop_speed = 40
    strafe_kPID  = (1, 0, 0),
    depth_kPID   = (1, 0, 0),
    depth_enable = False,

    class Meta:
        dynamic = [
            'ready_timeout', 'lost_timeout',
            'strafe_kPID', 'depth_kPID',
            'prop_speed', 'depth_enable'
        ]


class PipeConfirmer(messaging.MessageObserver):
    def __init__(self, auv, bins, centre_name='pipe', histogram_name='Hue', lower_threshold=0.05):
        messaging.MessageObserver.__init__(self)
        self.auv = auv
        self.strafeControl = PIDController((1,0,0))
        self.centre_name = centre_name
        self.histogram_name = histogram_name
        self.bins = bins
        self.intensity = MovingAverage(side= 'lower', tolerance=lower_threshold, maxcount=5, st_multiplier=2.5, st_on = 1)
        # required events
        self.intensityTriggered = threading.Event()
        self.linesTriggered = threading.Event()

    def onCentreMessage(self, m):
        if m.name == self.centre_name:
            # to get a good view we might need to move over the pipe
            # this assumes we're roughly in line with the pipe already
            # TODO: use the lines message to check if we need to correct
            # the alignment as well
            strafe = strafeControl.update(m.x - 0.5)
            debug('PipelineConfirmer: Set strafe: %i' % (int(strafe))
            self.auv.strafe(int(strafe))

    def onHistogramMessage(self, m):
        if m.name == self.histogram_name:
            # collate all the bins of interest to measure their
            # combined intensity
            collectedBins = []
            for bin in self.bins:
                collectedBins.append(m.bins[bin])
            # average out the intensity over time
            self.intensity.update(sum(collectedBins))
            if self.intensity.trigger > 10:
                self.intensityTrigger.set()
            else:
                self.intensityTrigger.clear()

    def onLinesMessage(self, m):
        parallelLines = 0;
        for i, line1 in enumerate(m.lines):
            for j, line2 in enumerate(m.lines):
                if degrees(abs(line1.angle - line2.angle)) < 15 and i != j:
                    parallelLines++
                    
        if(parallelLines >= 2):
            self.linesTrigger.set()
        else:
            self.linesTrigger.clear()


    def confirm(self, time = 10):
        # give it some time to confirm a sighting
        if(self.intensityTrigger.wait(time)):
            return self.linesTrigger.is_set()
        return False;
        


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
        
        # controllers for staying above the pipe
        self.depthControl = PIDController(self.options.depth_kPID)
        self.strafeControl = PIDController(self.options.strafe_kPID)

    def loadPipeline(self):
        self.__pl.load(Options.pipeline_file)

    def onLinesMessage(self, m):
        if m.name != self.options.lines_name:
            debug('follow pipe: ignoring lines message %s != %s' % (m.name, self.options.lines_name))
            return
        if len(m.lines):
            #
            # Adjust the AUv's bearing to align with the pipe
            # calculate the average angle of the lines (assumes good line finding)
            angle = sum([x.angle for x in m.lines])/len(m.lines)
            
            # calculate the bearing of the pipe (relative to the sub),
            # mod 180 as we dont want to accidentally turn the sub around
            corrected_angle=90-degrees(angle)%180
            debug('follow pipe: mean lines direction: %g (uncorrected=%g)' % (corrected_angle, angle))
            
            # work out the actual bearing of the pipe using the current bearing of the AUV
            current_bearing = self.auv.getBearing()
            if current_bearing: #watch out for none bearings
                self.auv.bearing((current_bearing-corrected_angle)%360) #- as angle is opposite direction to bearing
            
            #
            # Adjust the depth of the AUV acording to the width of the pipe in the image 
            # we can only calculate width if we have 2 lines, and dont bother if the angle is too different
            if len(m.lines) >= 2 and degrees(abs(m.lines[0].angle-m.lines[1].angle)) < 15 and self.depth_enable:
                xpos = []
                ypos = []
                for r in m.lines:
                    xpos.append(r.centre.x)
                    ypos.append(r.centre.y)
                # modulus of the cross product of the delta position of centres of 2 lines, and a unit vector of a line
                width = abs(sin(angle)*(max(xpos)-min(xpos))-cos(angle)*(max(ypos)-min(ypos))) 
                debug('follow pipe: estimated pipe width: %g' % width)
                
                # update the PID controller to get the required change in target depth
                dive = self.depthControl.update(width - self.options.target_width)
                if self.auv.current_depth: #again, none depths
                    self.auv.depth(self.auv.current_depth + dive)
                else:
                    dive = 0
                info('Angle: %f, Pipe/Image ratio: %f, Turn: %f, Change in depth: %f' %(angle, width, corrected_angle, dive))
            else:
                info('Angle: %f, Turn: %f, Not enough lines or lines to different for depth estimate or depth not enabled.' %(angle, corrected_angle))
            
            # set the flags that show if we're above the pipe
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
        
        strafe = self.strafeControl.update(m.x - 0.5)
        self.auv.strafe(strafe)
        info('pipe follow: strafing %i' %(strafe))
        
        # set the flag used for determining if we're above the pipe
        if m.x**2 + m.y**2 < self.options.centre_error**2:
            self.centred.set() #ie within circle radius centre error
        else:
            self.centered.clear()
        

    def onCornersMessage(self, m):
        #TODO: process message
        info("Corners message")


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
        
        # confirm we're above the pipe
        # it does this by looking for parallel lines and checking their is
        # a peak of intensity and the correct color
        confirmer = PipeConfirmer(self.auv, [11,12])
        self.node.addObserver(confirmer)
        sighted = PipeConfirmer.confirm(15) # give it 15 seconds
        self.node.removeObserver(confirmer)
        del confirmer
        if not sighted: return
        
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


