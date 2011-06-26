#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.pipeline as pipeline
import cauv.node
from cauv.debug import debug, info, warning, error

from AI_classes import aiScript, aiScriptOptions

import time
import optparse
import math
import traceback

class scriptOptions(aiScriptOptions):
    # TODO: need some mechanism of setting these from the AI framework?
    Do_Prop_Limit = 50  # max prop for forward/backward adjustment
    Camera_FOV = 60     # degrees
    Warn_Seconds_Between_Sights = 5
    Give_Up_Seconds_Between_Sights = 30
    Node_Name = "py-CrcB"
    Strafe_Speed = 50   # (int [-127,127]) controls strafe speed
    Buoy_Size = 0.2     # (float [0.0, 1.0]) controls distance from buoy. Units are field of view (fraction) that the buoy should fill
    Size_Control_kPD = (-30, 0)  # (Kp, Kd)
    Size_DError_Smoothing = 0.9  # 1 == never changes, 0 == no smoothing
    Angle_Control_kPD = (0.6, 0) # (Kp, Kd)
    Angle_DError_Smoothing = 0.9 # 1 == never changes, 0 == no smoothing    
    Pipeline_File = 'pipelines/circle_buoy.pipe'
    Load_Pipeline = 'default' # None, or name of running pipeline to load the image processing setup into

class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts)
        # self.node is set by aiProcess (base class of aiScript)
        self.node.join('processing')
        self.__pl = pipeline.Model(self.node, self.options.Load_Pipeline)
        self.__strafe_speed = self.options.Strafe_Speed
        self.__buoy_size = self.options.Buoy_Size
        self.last_size_err = None
        self.last_size_derr = 0
        self.size_derr_smoothing = self.options.Size_DError_Smoothing
        self.last_angle_err = None
        self.last_angle_derr = 0
        self.angle_derr_smoothing = 0.5
        self.time_last_seen = None
        self.__sizekpd = self.options.Size_Control_kPD
        self.__anglekpd = self.options.Angle_Control_kPD
    
    def loadPipeline(self):
        self.__pl.load(self.options.Pipeline_File)

    def onCirclesMessage(self, m):
        if str(m.name) == 'buoy':
            debug('Received circles message: %s' % str(m), 4)
            mean_circle_position = 0.5
            mean_circle_radius = 1
            num_circles = len(m.circles)
            if num_circles != 0:
                mean_circle_position = 0
                mean_circle_radius = 0
                for circle in m.circles:
                    mean_circle_position += circle.centre.x
                    mean_circle_radius += circle.radius
                mean_circle_position /= num_circles
                mean_circle_radius /= num_circles
                debug('Buoy at %s %s' % (mean_circle_position,
                                         mean_circle_radius))
                self.actOnBuoy(mean_circle_position,
                               mean_circle_radius)
            else:
                debug('No circles!', 3)
        else:
            debug('Ignoring circles message: %s' % str(m))
    
    def getBearingNotNone(self):
        b = self.auv.getBearing()
        if b is None:
            warning('no current bearing available')
            b = 0
        return b

    def actOnBuoy(self, centre, radius):
        now = time.time()
        if self.time_last_seen is not None and \
           now - self.time_last_seen > self.options.Warn_Seconds_Between_Sights:
            info('picked up buoy again')
         
        pos_err = centre - 0.5
        if pos_err < -0.5 or pos_err > 0.5:
            warning('buoy outside image: %g' % pos_err)
        #
        #          FOV /     - 0.5
        #             /      |
        #            /       |
        #           /        |
        #          /  buoy   |
        #         /    o     - pos_err
        #        /   /       |
        #       /  /         |
        #      / /           |
        #     // ) angle_err |
        # |->/- - - - -      -  0
        #         
        #    |<------------->|
        #       = 0.5 / sin(FOV/2) = plane_dist
        #
        plane_dist = 0.5 / math.sin(math.radians(self.options.Camera_FOV)/2.0)
        angle_err = math.degrees(math.asin(pos_err / plane_dist)) 
        angle_derr = 0
        if self.last_angle_err is not None:
            new_angle_derr = (angle_err - self.last_angle_err[1]) / (now - self.last_angle_err[0])
            angle_derr = self.angle_derr_smoothing * angle_derr +\
                         (1 - self.angle_derr_smoothing) * new_angle_derr
        self.last_angle_derr = angle_derr
        self.last_angle_err = (now, angle_err)
        turn_to = self.getBearingNotNone() +\
                  angle_err * self.__anglekpd[0] +\
                  angle_derr * self.__anglekpd[1]
        self.auv.bearing(turn_to)

        size_err = radius - self.__buoy_size
        size_derr = 0
        if self.last_size_err is not None:
            new_size_derr = (size_err - self.last_size_err[1]) / (now - self.last_size_err[0])
            size_derr = self.size_derr_smoothing * size_derr +\
                        (1 - self.size_derr_smoothing) * new_size_derr
        self.last_size_derr = size_derr
        self.last_size_err = (now, size_err)
        do_prop = self.__sizekpd[0] * size_err +\
                  self.__sizekpd[1] * size_derr
        if do_prop > self.options.Do_Prop_Limit:
            do_prop = self.options.Do_Prop_Limit
        if do_prop < -self.options.Do_Prop_Limit:
            do_prop = -self.options.Do_Prop_Limit
        debug('angle (e=%g, de=%g) size (e=%g, de=%g)' % (angle_err, angle_derr, size_err, size_derr))            
        debug('turn to %g, prop to %s' % (turn_to, do_prop))
        self.auv.prop(int(round(do_prop)))
        self.time_last_seen = now

    def run(self):
        if self.options.Load_Pipeline is not None:
            saved_pipeline_state = self.__pl.get()
            self.loadPipeline()
        start_bearing = self.auv.getBearing()
        entered_quarters = [False, False, False, False]
        exit_status = 0
        info('Waiting for circles...')
        try:
            while False in entered_quarters:
                time.sleep(0.5)
                time_since_seen = 0
                if self.time_last_seen is not None:
                    time_since_seen = time.time() - self.time_last_seen
                    if time_since_seen > self.options.Warn_Seconds_Between_Sights:
                        warning('cannot see buoy: last seen %g seconds ago' %
                                time_since_seen)
                    if time_since_seen > self.options.Give_Up_Seconds_Between_Sights:
                        # TODO: arrange for this sort of thing to be logged in
                        # the competition log
                        raise Exception('lost the buoy: giving up!')
                if self.auv.getBearing() > -180 and self.auv.getBearing() < -90:
                    entered_quarters[3] = True
                if self.auv.getBearing() > -90 and self.auv.getBearing() < 0:
                    entered_quarters[2] = True
                if self.auv.getBearing() > 0 and self.auv.getBearing() < 90:
                    entered_quarters[0] = True
                if self.auv.getBearing() > 90 and self.auv.getBearing() < 180:
                    entered_quarters[1] = True
                if self.auv.getBearing() > 180 and self.auv.getBearing() < 270:
                    entered_quarters[2] = True
                if self.auv.getBearing() > 270 and self.auv.getBearing() < 360:
                    entered_quarters[3] = True
            while self.auv.getBearing() < start_bearing:
                info('Waiting for final completion...')
                time.sleep(0.5)
        except:
            exit_status = 1
            error(traceback.format_exc())
        finally:
            info('Stopping...')
            self.auv.stop()
        info('Complete!')
        # restore pipeline that was running before
        if self.options.Load_Pipeline is not None:
            self.__pl.set(saved_pipeline_state)
        self.notify_exit(exit_status)

