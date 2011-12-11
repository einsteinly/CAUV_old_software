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
    follow_cam_file  = 'river-edges.pipe'
    lines_name = 'cam'
    #Timeouts
    ready_timeout = 30
    lost_timeout = 15
    # Calibration
    centre_error = 0.1 # of image
    align_error  = 10   # degrees 
    # Control
    prop_speed = 100
    max_search_angle = 50
    search_angle_increment = 5
    strafe_kPID  = (-300, 0, 0)

    
    class Meta:
        dynamic = [
            'ready_timeout', 'lost_timeout',
            'strafe_kPID', 'centre_error',
            'prop_speed', 'align_error'
        ]


class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts) 
        self.node.join("processing")
        
        # parameters to say if the auv is in the middle and aligned to the pipe
        self.centred = threading.Event()
        self.aligned = threading.Event()
        self.ready = threading.Event()
        self.detected = threading.Event()
        
        # controllers for staying above the pipe
        self.strafeControl = PIDController(self.options.strafe_kPID)

        self.yAverage = TimeAverage(1)
        self.last_detect_time = 0

    def onLinesMessage(self, m):
        if m.name != self.options.lines_name:
            debug('Cam follow: ignoring lines message %s != %s' % (m.name, self.options.lines_name))
            return

        if len(m.lines):                
            detected=False
            
            for i in range(len(m.lines)):
                for j in range(i):
                    #Filter out the lines that are vaguly pointing up
                    if abs(degrees(m.lines[i].angle)+90)<30 and abs(degrees(m.lines[j].angle)+90)<30:
                        #debug('got straigh lines')

                        #Filter out the lines that are paralle to each other by pairwise comparision
                        if degrees(abs(m.lines[i].angle - m.lines[j].angle)%180)<30:
                            #debug('got paralle')

                            #Filter out the lines that are too close apart
                            if ((m.lines[i].centre.x - m.lines[j].centre.x)**2+(m.lines[i].centre.y - m.lines[j].centre.y)**2)**0.5>0.3:
                                #debug('got apart')

                                #Also check if the AUV is in between the two lines
                                if max(m.lines[i].centre.x, m.lines[j].centre.x)>0.40 and min(m.lines[i].centre.x, m.lines[j].centre.x)<0.60:
                                    #debug('got middle')
                                    edges=[m.lines[i], m.lines[j]]
                                    detected = True
                                    break

                #Break out of the outer for loop
                if detected is True:
                    break         
            self.detected.clear()
        
            #Don't do anything else if the river edge is not detected
            if detected is True:            
                if self.auv.getBearing():  #In case it is none type
                    self.auv.bearing(self.auv.getBearing())   #Stop oscillating if it is
                self.detected.set()

                info('Cam follow: River edge detected')
                debug('Cam follow: Angle: %f, %F' %(degrees(edges[0].angle), degrees(edges[1].angle)))
                debug('Cam follow: Posistion: %f, %f' %(edges[0].centre.x, edges[1].centre.x))
                debug('Cam follow: Apart: %f' %((edges[0].centre.x-edges[1].centre.x)**2+(edges[0].centre.y-edges[1].centre.y)**2)**0.5)

                        
                #Set the time stamp
                self.last_detect_time = time.time()

                #
                # Adjust the AUV's bearing to align with River Cam by line detection of the bank
                # calculate the average angle of the lines (assumes good line finding)
                angle = degrees(sum([x.angle for x in edges])/len(edges))
                
                # calculate the bearing of River Cam (relative to the sub),
                # mod 180 as we dont want to accidentally turn the sub around
                corrected_angle=(90+angle)          


                if abs(corrected_angle) < self.options.align_error: 
                    self.aligned.set()

                #Align the AUV to River Cam if the error is outside of tolerance
                else: 
                    current_bearing = self.auv.getBearing()
                    if current_bearing: #watch out for none bearings
                        self.auv.bearing((current_bearing+corrected_angle)%360) 
                        self.aligned.clear()
                        info('Cam follow: Adjusting Bearing by: %g (uncorrected=%g)' % (corrected_angle, angle))


                #Centre the AUV in the middle of River Cam
                if self.aligned.is_set():                        #Only centre if the AUV is aligned
                    cam_centre=(edges[0].centre.x+edges[1].centre.x)/2
                    centre_err=cam_centre - 0.5

                    debug('Cam follow: Centre error: %f' %centre_err)
                    if abs(centre_err) < self.options.centre_error:
                        info('Cam follow: AUV is at the centre')
                        self.centred.set()
                    else:

                        strafe_demand = int(self.strafeControl.update(centre_err))
                        if strafe_demand < -127: strafe_demand = -127
                        elif strafe_demand > 127: strafe_demand = 127
                        self.auv.strafe(strafe_demand)
                        info('Cam follow: strafing %i' %(strafe_demand))
                        self.centred.clear()


                # set the flags that show if we're aligned and centred at the Cam
                if self.centred.is_set() and self.aligned.is_set():
                    self.ready.set()
                    info('Cam follow: AUV is centred and aligned')
                    self.log('Cam follow: Done aligning and centering.')
                else:
                    debug('Cam follow: centred:%s aligned:%s' %
                            (self.centred.is_set(), self.aligned.is_set())
                    )
                    self.ready.clear()



    def run(self):
        self.log('Cam follow: Initiating following river cam.')
        follow_cam_file = self.options.follow_cam_file
        #self.request_pl(follow_cam_file)
    
        while True:

            #Check if the river edge is detected recently
            if (time.time()-self.last_detect_time)<self.options.lost_timeout:
                #Start moving as soon as alignment and centre is ready
                #self.ready.wait()
                self.auv.prop(self.options.prop_speed)        
                time.sleep(self.options.lost_timeout)        #Sleep until the start of next cycle
            else:
                self.auv.stop()        #Stop immediatly if the River edge is lost for too long
                self.detected.clear()
                info('Cam follow: The edge of River Cam is lost, trying to find it again.') 
                if self.auv.getBearing():  #In case it is none type     
                    current_bearing = self.auv.getBearing()   
                    for angle in range(self.options.search_angle_increment, self.options.max_search_angle, self.options.search_angle_increment):
                        if self.detected.is_set() is False:
                            info('oscillating by %i degrees' %angle)
                            self.auv.bearingAndWait((current_bearing+angle)%360) #Perform 1 revolution of self rotation until the AUV is aligned again
                        if self.detected.is_set() is False:
                            self.auv.bearingAndWait((current_bearing-angle)%360)
                        if self.detected.is_set() is True:
                            break

                else:
                    time.sleep(1)
            
    def stop(self):
        self.auv.prop(0)
        self.drop_pl(follow_cam_file)
        self.log('Cam follow: Stopping followoing River Cam.')
        info('Cam follow: Stopping River Cam following')


if __name__=="__main__":
        cam_follower=script()
        cam_follower.run()

