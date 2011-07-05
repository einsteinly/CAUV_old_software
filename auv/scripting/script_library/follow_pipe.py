#!/usr/bin/env python

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
    confirm_pipeline_file = 'confirm_pipe.pipe'
    follow_pipeline_file  = 'follow_pipe.pipe'
    centre_name = 'pipe'
    lines_name = 'pipe'
    histogram_name = 'pipe'
    colour_bins = (3,4,5)
    #Timeouts
    confirm_timeout = 15
    ready_timeout = 200
    lost_timeout = 2
    # Calibration
    target_width = 0.2 # of image
    width_error   = 0.1 # of image
    centre_error = 0.1 # of image
    align_error  = 5   # degrees 
    average_time = 1   # seconds
    intensity_trigger = 0.10
    # Control
    prop_speed = 40
    strafe_kPID  = (100, 0, 0)
    depth_kPID   = (1, 0, 0)

    class Meta:
        dynamic = [
            'ready_timeout', 'lost_timeout',
            'strafe_kPID', 'depth_kPID',
            'prop_speed', 'average_time'
        ]


class PipeConfirmer(messaging.MessageObserver):
    def __init__(self, auv, options):
        messaging.MessageObserver.__init__(self)
        self.auv = auv
        self.options = options
        self.strafeControl = PIDController(self.options.strafe_kPID)
        self.intensity = TimeAverage(self.options.average_time)
        # required events
        self.intensityTrigger = threading.Event()
        self.linesTrigger = threading.Event()
        self.sighted = threading.Event()

    def onCentreMessage(self, m):
        if m.name == self.options.centre_name:
            # to get a good view we might need to move over the pipe
            # this assumes we're roughly in line with the pipe already
            # TODO: use the lines message to check if we need to correct
            # the alignment as well
            strafe = self.strafeControl.update(m.x - 0.5)
            debug('PipelineConfirmer: Set strafe: %i' % (int(strafe)))
            self.auv.strafe(int(strafe))

    def onHistogramMessage(self, m):
        if m.name == self.options.histogram_name:
            # collate all the bins of interest to measure their
            # combined intensity
            collectedBins = []
            for bin in self.options.colour_bins:
                collectedBins.append(m.bins[bin])
            # average out the intensity over time
            averageIntensity = self.intensity.update(sum(collectedBins))
            debug("Intensity trigger %f percent" % ((averageIntensity / self.options.intensity_trigger) * 100))
            if averageIntensity > self.options.intensity_trigger:
                self.intensityTrigger.set()
                if self.linesTrigger.is_set():
                    self.sighted.set()
                    info("Pipeline sighting confirmed")
            else:
                self.intensityTrigger.clear()
                

    def onLinesMessage(self, m):
        if m.name == self.options.lines_name:
            parallelLines = 0;
            for i, line1 in enumerate(m.lines):
                for j, line2 in enumerate(m.lines):
                    if degrees(abs(line1.angle - line2.angle)) < 15 and i != j:
                        parallelLines += 1
                
            debug("Parallel Lines: %d" % (parallelLines))
            if(parallelLines >= 2):
                self.linesTrigger.set()
            else:
                self.linesTrigger.clear()


    def confirm(self, time = 10):
        # give it some time to confirm a sighting
        self.sighted.wait(time)
        return self.sighted.is_set()

class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts) 
        self.node.join("processing")
        
        # parameters to say if the auv is above the pipe
        self.centred = threading.Event()
        self.aligned = threading.Event()
        self.depthed = threading.Event()
        self.corners = threading.Event()
        self.confirmed = threading.Event()
        self.ready = threading.Event()
        
        # controllers for staying above the pipe
        self.depthControl = PIDController(self.options.depth_kPID)
        self.strafeControl = PIDController(self.options.strafe_kPID)

    def onLinesMessage(self, m):
        if not self.confirmed.is_set(): return
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
            corrected_angle=90-degrees(angle)%180
            debug('follow pipe: mean lines direction: %g (uncorrected=%g)' % (corrected_angle, angle))
            
            # work out the actual bearing of the pipe using the current bearing of the AUV
            current_bearing = self.auv.getBearing()
            if current_bearing: #watch out for none bearings
                self.auv.bearing((current_bearing-corrected_angle)%360) #- as angle is opposite direction to bearing
            
            if abs(corrected_angle) < self.options.align_error: self.aligned.set()
            else: self.aligned.clear()
            
            
            #
            # Adjust the depth of the AUV acording to the width of the pipe in the image 
            # we can only calculate width if we have 2 lines, and dont bother if the angle is too different
            if len(m.lines) >= 2 and degrees(abs(m.lines[0].angle-m.lines[1].angle)) < 15:
                xpos = []
                ypos = []
                for r in m.lines:
                    xpos.append(r.centre.x)
                    ypos.append(r.centre.y)
                # modulus of the cross product of the delta position of centres of 2 lines, and a unit vector of a line
                width = abs(sin(angle)*(max(xpos)-min(xpos))-cos(angle)*(max(ypos)-min(ypos))) 
                debug('follow pipe: estimated pipe width: %g' % width)
                
                # update the PID controller to get the required change in target depth
                width_error = width - self.options.target_width
                debug("Width error = %f" % (width_error))
                dive = self.depthControl.update(width_error)
                if self.auv.current_depth: #again, none depths
                    self.auv.depth(self.auv.current_depth + dive)
                
                if width_error < self.options.width_error: self.depthed.set()
                else: self.depthed.clear()
            
                
            # set the flags that show if we're above the pipe
            if self.centred.is_set() and self.aligned.is_set() and self.depthed.is_set(): self.ready.set()
            else: self.ready.clear()


    def onCentreMessage(self, m):
        if not self.confirmed.is_set(): return
        if m.name != self.options.centre_name:
            debug('follow pipe: ignoring centre message %s != %s' % (m.name, self.options.centre_name))
            return
        
        
        info('Centre error: %f' %(m.x - 0.5))
        strafe = int(self.strafeControl.update(m.x - 0.5))
        self.auv.strafe(strafe)
        info('pipe follow: strafing %i' %(strafe))
        
        # set the flag used for determining if we're above the pipe
        if m.x**2 + m.y**2 < self.options.centre_error**2:
            self.centred.set() #ie within circle radius centre error
        else:
            self.centred.clear()
        

    def onCornersMessage(self, m):
        if not self.confirmed.is_set(): return
        #TODO: process message
        info("Corners message TODO: process this and set self.corners")


    def followPipeUntil(self, condition):
        while not condition.is_set():
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
                time.sleep(1)
        
        return True
  

    def run(self):
        # first we need to check we've actually found the pipe
        # the detector doesn't really look for the pipe it just looks
        # for yellow things in the downward camera, so there will be
        # a few false positives
        
        # confirm we're above the pipe
        # it does this by looking for parallel lines and checking their is
        # a peak of intensity in the correct color
        conf_pipe_file = self.options.confirm_pipeline_file
        self.request_pl(conf_pipe_file)
        confirmer = PipeConfirmer(
            self.auv,
            self.options
        )
        self.node.addObserver(confirmer)
        if(confirmer.confirm(self.options.confirm_timeout)):
            info("Pipeline confirmed")
            self.confirmed.set()
        self.node.removeObserver(confirmer)
        del confirmer
        self.drop_pl(conf_pipe_file)
        if not self.confirmed.is_set():
            error("Pipeline sighting could not be confirmed. Abandoning")
            self.notify_exit('ABANDONED')
            return
        info("Pipeline sighting was confirmed.")

        
        follow_pipe_file = self.options.follow_pipeline_file
        self.request_pl(follow_pipe_file)
            
        # now we wait for messages allowing us to work out how to align with
        # the pipe, but if this is taking too long then just give up as we've
        # probably drifted away from the pipe
        debug('Waiting for ready...')
        self.ready.wait(self.options.ready_timeout)
        if self.ready.is_set():
            error("Took too long to become ready, aborting")
            self.drop_pl(follow_pipe_file)
            self.notify_exit('ABORT')            
            return #timeout
        
        
        for i in range(3):
            # follow the pipe along until the end (when we can see the corners)
            if not self.followPipeUntil(self.corners):
                error("Pipeline lost on pass %d" %(i,));
                self.drop_pl(follow_pipe_file)
                self.notify_exit('LOST')
                return
            
            # turn 180
            info("Reached end of pass. Doing 180")
            self.auv.prop(0)
            self.auv.bearing((self.auv.getBearing()-180)%360)
        
        self.drop_pl(follow_pipe_file)

        info('Finished pipe following')
        self.notify_exit('SUCCESS')


