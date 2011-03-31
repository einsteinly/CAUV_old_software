#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.pipeline as pipeline
import cauv.node
from cauv.debug import debug, info, warning, error

import time
import optparse
import math

class CircleBuoyDefaults:
    # default options
    Do_Prop_Limit = 50  # max prop for forward/backward adjustment
    Camera_FOV = 60     # degrees
    Warn_Seconds_Between_Sights = 5
    Node_Name = "py-CrcB"
    Strafe_Speed = 20
    Buoy_Size = 0.2     # as fraction of camera FOV at desired distance
    Size_Control_kPD = (300, 0)


class BuoyCircleObserver(msg.MessageObserver):
    def __init__(self, cauv_node, auv, strafe_speed, buoy_size,
                 size_kpd=CircleBuoyDefaults.Size_Control_kPD):
        msg.MessageObserver.__init__(self)
        self.__node = cauv_node
        self.__node.join('processing')
        self.__node.addObserver(self)
        self.__auv = auv
        self.__pl = pipeline.Model(self.__node)
        self.__strafe_speed = strafe_speed
        self.__buoy_size = buoy_size
        self.last_size_err = None
        self.time_last_seen = None
        self.__sizekpd = size_kpd
    
    def loadPipeline(self):
        self.__pl.load('pipelines/circle_buoy.pipe')

    def onCirclesMessage(self, m):
        if str(m.name) == 'buoy':
            debug('Received circles message: %s' % str(m))
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
                debug('No circles!')
        else:
            debug('Ignoring circles message: %s' % str(m))
    
    def getBearingNotNone(self):
        b = self.__auv.getBearing()
        if b is None:
            warning('no current bearing available')
            b = 0
        return b

    def actOnBuoy(self, centre, radius):
        now = time.time()
        if self.time_last_seen is not None and \
           now - self.time_last_seen > CircleBuoyDefaults.Warn_Seconds_Between_Sights:
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
        plane_dist = 0.5 / math.sin(math.radians(CircleBuoyDefaults.Camera_FOV)/2.0)
        angle_err = math.degrees(math.asin(pos_err / plane_dist))
        debug('angle error = %g' % angle_err)
        turn_to = self.getBearingNotNone() + angle_err
        debug('turning to %g' % turn_to)
        self.__auv.bearing(turn_to)

        size_err = radius - self.__buoy_size
        size_derr = 0
        if self.last_size_err is not None:
            size_derr = (size_err - self.last_size_err[1]) / (now - self.last_size_err[0])
        self.last_size_err = (now, size_err)
        do_prop = self.__sizekpd[0] * size_err +\
                  self.__sizekpd[1] * size_derr
        if do_prop > CircleBuoyDefaults.Do_Prop_Limit:
            do_prop = CircleBuoyDefaults.Do_Prop_Limit
        if do_prop < -CircleBuoyDefaults.Do_Prop_Limit:
            do_prop = -CircleBuoyDefaults.Do_Prop_Limit
        debug('setting prop: %s' % do_prop)
        self.__auv.prop(int(round(do_prop)))
        self.time_last_seen = now

    def run(self, load_pipeline=True):
        if load_pipeline:
            self.loadPipeline()
        start_bearing = self.__auv.getBearing()
        entered_quarters = [False, False, False, False]
        info('Waiting for circles...')
        try:
            while False in entered_quarters:
                time.sleep(0.5)
                time_since_seen = 0
                if self.time_last_seen is not None:
                    time_since_seen = time.time() - self.time_last_seen
                    if time_since_seen > CircleBuoyDefaults.Warn_Seconds_Between_Sights:
                        warning('cannot see buoy: last seen %g seconds ago' %
                                time_since_seen)
                if self.__auv.getBearing() > -180 and self.__auv.getBearing() < -90:
                    entered_quarters[3] = True
                if self.__auv.getBearing() > -90 and self.__auv.getBearing() < 0:
                    entered_quarters[2] = True
                if self.__auv.getBearing() > 0 and self.__auv.getBearing() < 90:
                    entered_quarters[0] = True
                if self.__auv.getBearing() > 90 and self.__auv.getBearing() < 180:
                    entered_quarters[1] = True
                if self.__auv.getBearing() > 180 and self.__auv.getBearing() < 270:
                    entered_quarters[2] = True
                if self.__auv.getBearing() > 270 and self.__auv.getBearing() < 360:
                    entered_quarters[3] = True
            while self.__auv.getBearing() < start_bearing:
                info('Waiting for final completion...')
                time.sleep(0.5)
        finally:
            info('Stopping...')
            self.__auv.stop()
        info('Complete!')
            


def runWithNode(cauv_node, auv, opts=None):
    # opts.strafe_speed (int [-127,127]) controls strafe speed
    # opts.buoy_size (float [0.0, 1.0]) controls distance from buoy. Units are
    # field of view that the buoy should fill
    info('circle_buoy.py main program')
    if auv is None:
        auv = control.AUV(cauv_node)

    b = BuoyCircleObserver(cauv_node, auv, opts.strafe_speed, opts.buoy_size)
    b.run(opts.do_load_pipeline)

    info('circle_buoy.py complete')



def runStandalone(node_name = 'py-crcb', opts=None):
    node = cauv.node.Node('py-crcb')
    runWithNode(node, None, opts)

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option("-n", "--name", dest="name",
        default=CircleBuoyDefaults.Node_Name, help="CAUV Node name")
    p.add_option('-s', "--strafe-speed", dest="strafe_speed",
        default=CircleBuoyDefaults.Strafe_Speed, type=int)
    p.add_option('-b', "--buoy-size", dest="buoy_size",
        default=CircleBuoyDefaults.Buoy_Size, type=int)
    p.add_option('-L', "--no-load-pipeline", dest="do_load_pipeline",
        default=True, action='store_false')
    opts, args = p.parse_args()

    runStandalone(opts.name, opts)

    

