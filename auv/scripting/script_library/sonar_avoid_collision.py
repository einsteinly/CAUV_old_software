#!/usr/bin/env python2.7

# Standard Library
import math
import time

# CAUV:
from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, info, warning, error
import cauv.messaging as msg
import cauv.pipeline  as pipeline

from utils.timeutils import RelativeTimeCapability


class scriptOptions(aiScriptOptions):
    Sonar_Range = 35
    KeyPoints_Name = 'sonar_local_maxima_bearing_range'
    Too_Close_Static = 0.2  # metres
    Too_Close_Threshold = 12 # higher = less likely to detect collision from close objects
    Dynamic_Vehicle_Width = 0.2
    Max_Velocity = 2 # metres per second
    Max_Range = 4    # maximum range to consider targets
    Run_Away_Time = 1 # seconds

class script(aiScript, RelativeTimeCapability):
    def __init__(self, *arg, **kwargs):
        aiScript.__init__(self, *arg, **kwargs)
        RelativeTimeCapability.__init__(self)
        self.Keypoints_Name = self.options.KeyPoints_Name
        self.last_keypoints = None
        self.nearby_detected = 0
        self.last_time = 0
        self.time_detected = None
    
    def setDetected(self):
        self.detected = True
        self.time_detected = self.relativeTime()

    def clearDetected(self):
        self.detected = False

    def checkStaticCollision(self, keypoints):
        nearby = 0
        thr = self.options.Too_Close_Threshold
        for kp in keypoints:
            if kp.pt.y < self.options.Too_Close_Static:
                nearby += 1
        if nearby == 0 and self.nearby_detected > 0:
            if self.nearby_detected > 3*thr:
                self.nearby_detected = 3*thr
            self.nearby_detected /= 1.2
        else:
            self.nearby_detected += nearby
        msg = 'nearby sonar responses: %s/%s' % (self.nearby_detected, thr)
        if self.nearby_detected > thr:
            info(msg)
            self.setDetected()
        else:
            debug(msg)
    
    def checkDynamicCollisions(self, kps_last, kps_now, dt):
        # we're dealing with a small number of keypoints so this is okay!
        kp_vectors = []
        moving_away = 0
        moving_slowly = 0
        moving_quickly = 0
        missing_left = 0
        missing_right = 0
        hit = 0
        for kp in kps_now:
            # associate each keypoint with closest previous keypoint:
            closest_kp = kps_last[0]
            closest_kp_sqdist = ((kps_last[0].pt.x - kp.pt.x)**2 + \
                                 (kps_last[0].pt.y - kp.pt.y)**2)
            for last in kps_last:
                sqdist = ((last.pt.x - kp.pt.x)**2 + \
                          (last.pt.y - kp.pt.y)**2)
                if sqdist < closest_kp_sqdist:
                    closest_kp_sqdist = sqdist
                    closest_kp = last
            (vx, vy) = ((kp.pt.x-closest_kp.pt.x)/dt,
                        (kp.pt.y-closest_kp.pt.y)/dt)
            if vy >= 0:
                #debug('kp moving away: vx=%s %s' % (vx, kp.pt))
                moving_away += 1
                continue
            # +---------------------->y
            # |           o   |       60
            # |\        /     |
            # ||      o       |
            # 0|vehicle width - 0
            # ||   .          |
            # |/ .            |
            # |.              |
            #.x               - +
            #
            time_to_collision = kp.pt.y / vy
            if time_to_collision > min((3*dt,1.0)):
                #debug('kp moving slowly: vy=%s: %s' % (vy, kp.pt))
                moving_slowly += 1
                continue
            if vx**2 + vy**2 > self.options.Max_Velocity**2:
                #debug('kp moving too quickly (probably noise): %s,%s: %s' % (vx,vy,kp.pt))
                moving_quickly += 1
                continue
            x_pos_at_collision = kp.pt.x + vx * time_to_collision 
            vehicle_width = self.options.Dynamic_Vehicle_Width
            vehicle_x_min = -0.5*vehicle_width
            vehicle_x_max =  0.5*vehicle_width
            if x_pos_at_collision < vehicle_x_min:
                #debug('kp missing right: %s' % kp.pt)
                missing_right += 1
                continue
            if x_pos_at_collision > vehicle_x_max:
                #debug('kp missing right(?): %s' % kp.pt)
                missing_left += 1
                continue
            debug('kp hit!: %s,%s: %s' % (vx,vy,kp.pt))
            hit += 1
        debug('hit: %d, miss: away=%d slowly=%d tooquick=%d left=%d right=%d' % (
            hit, moving_away, moving_slowly, moving_quickly, missing_left, missing_right
        ))
        if hit > 0:
            info('%d hits projected' % hit)
            self.setDetected()
    
    def convertKPsToMetres(self, kps_bearingrange):
        # because the localmaxima node uses x and y to store bearing range,
        # which is somewhat wrong, but nevermind:
        # Also here we discard keypoints that are a long way away (> Max_Range)
        Max_Range = self.options.Max_Range
        r = []
        for kp in kps_bearingrange:
            bearing = kp.pt.x
            range = kp.pt.y
            if range > Max_Range:
                continue
            t = msg.KeyPoint(
                msg.floatXY(-range*math.sin(bearing), range*math.cos(bearing)),
                kp.size,
                kp.angle,
                kp.response,
                kp.octave,
                kp.class_id
            )
            r.append(t)
        return r

    def onKeyPointsMessage(self, m):
        if m.name != self.Keypoints_Name:
            return
        kps = self.convertKPsToMetres(m.keypoints)
        self.checkStaticCollision(kps)
        if self.last_keypoints is None or not len(self.last_keypoints):
            self.last_keypoints = kps
        else:
            now = self.relativeTime()
            self.checkDynamicCollisions(self.last_keypoints, kps, now - self.last_time)
            self.last_keypoints = kps
            self.last_time = now
            
    def run(self):
        self.node.join('processing')
        self.request_pl('sonar_collisions')
        while True:
            time.sleep(0.1)
            if self.time_detected is not None and\
               self.relativeTime() - self.time_detected < self.options.Run_Away_Time:
                info('running away!')
                if not self.in_control.is_set():
                    self.request_control_and_wait(1, control_timeout=5)
                self.auv.prop(-127)
            else:
                if self.time_detected is not None and \
                   self.relativeTime() - self.time_detected < self.options.Run_Away_Time + 0.5:
                    self.auv.prop(0)
                    self.drop_control()
                self.clearDetected()

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    opts, args = p.parse_known_args()
    
    a = SonarCollisionAvoider(detectorOptions())
    try:
        a.run()
    finally:
        a.die()

