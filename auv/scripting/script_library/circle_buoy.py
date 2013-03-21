#!/usr/bin/env python2.7

import cauv
import cauv.messaging as msg
from cauv.debug import debug, info, warning, error

import AI

import time
import math
import traceback

from utils.control import expWindow, PIDController

class CircleBuoy(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.Do_Prop_Limit = 10  # max prop for forward/backward adjustment
            self.Camera_FOV = 90     # degrees
            self.Warn_Seconds_Between_Sights = 5
            self.Give_Up_Seconds_Between_Sights = 30
            self.Depth_Ceiling = 0.8
            self.Strafe_Speed = 10   # (int [-127,127]) controls strafe speed
            self.Buoy_Size = 0.15     # (float [0.0, 1.0]) controls distance from buoy. Units are field of view (fraction) that the buoy should fill
            self.size_ctrl = PIDController((-30.0, -1.0, 0.0), expWindow(5, 0.6), 80.0)
            self.angle_ctrl = PIDController((0.2, 0.0, 0.0), expWindow(5, 0.6), 1e30)
            self.depth_ctrl = PIDController((0.08, 0.0, 0.0), expWindow(5, 0.6), 200)
            self.TotalRightAnglesToTurn = 4

    def telemetry(self, name, value):
        self.node.send(msg.GraphableMessage(name, value))

    def onCirclesMessage(self, m):
        if str(m.name) == 'buoy':
            debug('Received circles message: {}'.format(m), 2)
            mean_circle_position = [0.5, 0.5]
            mean_circle_radius = 1
            num_circles = len(m.circles)
            if num_circles != 0:
                mean_circle_position = [0,0]
                mean_circle_radius = 0
                for circle in m.circles:
                    mean_circle_position[0] += circle.centre.x
                    mean_circle_position[1] += circle.centre.y
                    mean_circle_radius += circle.radius
                mean_circle_position[0] /= num_circles
                mean_circle_position[1] /= num_circles
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
    
    def getDepthNotNone(self):
        d = self.auv.current_depth
        if d is None:
            warning('no current depth available')
            d = 0
        return d

    def actOnBuoy(self, centre, radius):
        now = time.time()
        if self.time_last_seen is not None and \
           now - self.time_last_seen > self.options.Warn_Seconds_Between_Sights:
            info('picked up buoy again')
             
        pos_err_x = centre[0] - 0.5
        pos_err_y = centre[1] - 0.5
        if pos_err_x < -0.5 or pos_err_x > 0.5: warning('buoy outside image (x): %g' % pos_err_x)
        if pos_err_y < -0.5 or pos_err_y > 0.5: warning('buoy outside image (y): %g' % pos_err_y)
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
        angle_err = math.degrees(math.asin(pos_err_x / plane_dist)) 
        depth_err = pos_err_y
        
        turn_to = self.getBearingNotNone() + self.options.angle_ctrl.update(angle_err)
        self.auv.bearing(turn_to)

        depth_to = self.getDepthNotNone() + self.options.depth_ctrl.update(depth_err)
        if depth_to < self.options.Depth_Ceiling:
            warning('setting depth limited by ceiling at %gm' %
                self.options.Depth_Ceiling
            )
            depth_to = self.options.Depth_Ceiling
        self.auv.depth(depth_to)

        size_err = radius - self.options.Buoy_Size
        do_prop = self.options.size_ctrl.update(size_err)

        if do_prop > self.options.Do_Prop_Limit:
            do_prop = self.options.Do_Prop_Limit
        if do_prop < -self.options.Do_Prop_Limit:
            do_prop = -self.options.Do_Prop_Limit
        self.auv.prop(int(round(do_prop)))
        self.time_last_seen = now

        for name, controller in {"angle": self.options.angle_ctrl,
                                 "size": self.options.size_ctrl,
                                 "depth": self.options.depth_ctrl}.iteritems():
            debug('{} (e={c.err:.3g}, ie={c.ierr:.3g}, de={c.derr:.3g})'.format(name, c=controller))
        debug('current_depth = {:.3g}'.format(self.getDepthNotNone()))
        debug('turn to %g, prop to %s, dive to %g' % (turn_to, do_prop, depth_to))
        
    def run(self):
        self.load_pipeline('detect_buoy_sim')
        self.node.subMessage(msg.CirclesMessage())
        self.time_last_seen = None
        if not self.auv.current_bearing:
            time.sleep(1)
        start_bearing = self.auv.current_bearing
        total_right_angles_turned = 0
        info('Waiting for circles...')
        last_bearing = self.auv.current_bearing
        while total_right_angles_turned < self.options.TotalRightAnglesToTurn:
            time.sleep(1)
            time_since_seen = 0
            if self.time_last_seen is None:
                warning('Not seen buoy, waiting')
            else:
                time_since_seen = time.time() - self.time_last_seen
                if time_since_seen > self.options.Warn_Seconds_Between_Sights:
                    warning('cannot see buoy: last seen %g seconds ago' %
                            time_since_seen)
                    self.auv.strafe(0)
                else:
                    self.auv.strafe(self.options.Strafe_Speed)
                if time_since_seen > self.options.Give_Up_Seconds_Between_Sights:
                    self.log('Buoy Circling: lost sight of the buoy!')
                    return False
            #note travelling left, so turning clockwise ie bearing increasing
            if (self.auv.current_bearing != None):
                bearing_diff = abs(self.auv.current_bearing - last_bearing)
                if min(bearing_diff, 360-bearing_diff) > 90: #assume we don't turn to fast
                    last_bearing = (last_bearing+90)%360
                    total_right_angles_turned += 1
        self.log('Buoy Circling: completed successfully')
        return

Script = CircleBuoy

if __name__ == "__main__":
    CircleBuoy.entry()
