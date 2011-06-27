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

import utils.control

def expWindow(n, alpha):
    t = 1
    r = [t]
    for i in xrange(0,n):
        r.append(t * alpha)
    return tuple(r)

class scriptOptions(aiScriptOptions):
    Do_Prop_Limit = 50  # max prop for forward/backward adjustment
    Camera_FOV = 60     # degrees
    Warn_Seconds_Between_Sights = 5
    Give_Up_Seconds_Between_Sights = 30
    Node_Name = "py-CrcB"
    Strafe_Speed = 50   # (int [-127,127]) controls strafe speed
    Buoy_Size = 0.2     # (float [0.0, 1.0]) controls distance from buoy. Units are field of view (fraction) that the buoy should fill
    Size_Control_kPID = (-30, 0, 0)  # (Kp, Kd)
    Size_DError_Window = expWindow(5, 0.6)
    Size_Error_Clamp = 1e30
    Angle_Control_kPID = (0.6, 0, 0) # (Kp, Kd)
    Angle_DError_Window = expWindow(5, 0.6)
    Angle_Error_Clamp = 1e30
    
    Pipeline_File = 'pipelines/circle_buoy.pipe'
    Load_Pipeline = 'default' # None, or name of running pipeline to load the image processing setup into

    class Meta:
        dynamic = [
            'Do_Prop_Limit', 'Strafe_Speed', 'Buoy_Size', 'Size_Control_kPID',
            'Size_DError_Window', 'Size_Error_Clamp', 'Angle_Control_kPID',
            'Angle_DError_Window', 'Angle_Error_Clamp'
        ]


class Controller:
    def __init__(self):
        pass
    def update(self, value):
        return 0

class PIDController(Controller):
    def __init__(self, (Kp, Ki, Kd), d_window = (1), err_clamp = 1e9):
        # the most recent end of the window is the start
        self.err = 0
        self.last_time = None
        self.derrs = []
        self.ierr = 0
        self.derr = 0
        self.derr_window = d_window
        self.err_clamp = err_clamp
        if abs(sum(d_window)) < 1e-30:
            raise Exception('Bad Window')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
    
    def calcDErr(self):
        # apply self.derr_window over self.derrs
        renorm = sum(self.derr_window)
        keep_derrs = max(10, len(self.derr_window))
        self.derrs = self.derrs[-keep_derrs:]
        denormed = 0
        for i, derr in enumerate(self.derrs):
            if i == len(self.derr_window):
                break
            denormed += derr * self.derr_window[-i]
        return denormed / renorm
    
    def setKpid(self, (Kp, Ki, Kd)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
    def update(self, err):
        now = time.time()
        last_err = self.err
        self.err = err
        if self.last_time is not None:
            self.derrs.append((self.err - last_err) / (now - self.last_time))
        else:
            self.derrs.append(0)
        self.derr = self.calcDErr()
        
        self.last_time = now
        
        self.ierr += self.err
        
        if self.ierr > self.err_clamp:
            self.ierr = self.err_clamp
        elif self.ierr < -self.err_clamp:
            self.ierr = -self.err_clamp
    
        return self.Kp * self.err +\
               self.Ki * self.ierr +\
               self.Kd * self.derr

class script(aiScript):
    def __init__(self, script_name, opts):
        aiScript.__init__(self, script_name, opts)
        # self.node is set by aiProcess (base class of aiScript)
        self.node.join('processing')
        self.__pl = pipeline.Model(self.node, self.options.Load_Pipeline)
        self.__strafe_speed = self.options.Strafe_Speed
        self.__buoy_size = self.options.Buoy_Size
        self.size_pid = PIDController(
                self.options.Size_Control_kPID,
                self.options.Size_DError_Window,
                self.options.Size_Error_Clamp
        )
        #self.last_size_err = None
        #self.last_size_derr = 0
        #self.size_derr_smoothing = self.options.Size_DError_Smoothing
        self.angle_pid = PIDController(
                self.options.Angle_Control_kPID,
                self.options.Angle_DError_Window,
                self.options.Angle_Error_Clamp
        )
        #self.last_angle_err = None
        #self.last_angle_derr = 0
        #self.angle_derr_smoothing = 0.5
        self.time_last_seen = None
        #self.__sizekpd = self.options.Size_Control_kPD
        #self.__anglekpd = self.options.Angle_Control_kPD
    
    def reloadOptions(self):
        self.__strafe_speed = self.options.Strafe_Speed
        self.__buoy_size = self.options.Buoy_Size
        self.size_pid.setKpid(self.options.Size_Control_kPID)
        self.size_pid.derr_window = self.options.Size_DError_Window
        self.size_pid.err_clamp   = self.options.Size_Error_Clamp
        self.angle_pid.setKpid(self.options.Angle_Control_kPID)
        self.angle_pid.derr_window = self.options.Angle_DError_Window
        self.angle_pid.err_clamp   = self.options.Angle_Error_Clamp

    def optionChanged(self, option_name):
        self.reloadOptions()

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
        
        turn_to = self.getBearingNotNone() + self.angle_pid.update(angle_err)
        '''
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
        '''
        self.auv.bearing(turn_to)

        size_err = radius - self.__buoy_size
        do_prop = self.size_pid.update(size_err)
        '''
        size_derr = 0
        if self.last_size_err is not None:
            new_size_derr = (size_err - self.last_size_err[1]) / (now - self.last_size_err[0])
            size_derr = self.size_derr_smoothing * size_derr +\
                        (1 - self.size_derr_smoothing) * new_size_derr
        self.last_size_derr = size_derr
        self.last_size_err = (now, size_err)
        do_prop = self.__sizekpd[0] * size_err +\
                  self.__sizekpd[1] * size_derr
        '''
        if do_prop > self.options.Do_Prop_Limit:
            do_prop = self.options.Do_Prop_Limit
        if do_prop < -self.options.Do_Prop_Limit:
            do_prop = -self.options.Do_Prop_Limit
        debug('angle (e=%.3g, ie=%.3g de=%.3g) size (e=%.3g, ie=%.3g, de=%.3gg)' % (
            self.angle_pid.err, self.angle_pid.ierr, self.angle_pid.derr,
            self.size_pid.err, self.size_pid.ierr, self.size_pid.derr
        ))
        debug('turn to %g, prop to %s' % (turn_to, do_prop))
        self.auv.prop(int(round(do_prop)))
        #self.time_last_seen = now

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

