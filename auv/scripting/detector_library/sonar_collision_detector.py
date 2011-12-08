#!/usr/bin/env python2.7

# Standard Library
import math

# CAUV:
from AI_classes import aiDetector, aiDetectorOptions
from cauv.debug import debug, info, warning, error

import cauv.pipeline as pipeline

from utils.timeutils import RelativeTimeCapability


class detectorOptions(aiDetectorOptions):
    Sonar_Range = 35
    KeyPoints_Name = 'sonar_local_maxima'
    Metres_Per_Px = 35.0 / 400
    Too_Close_Static = 0.5 # metres
    Too_Close_Threshold = 3 # higher = less likely to detect collision from close objects
    Dynamic_Vehicle_Width = 1

class detector(aiDetector, RelativeTimeCapability):
    def __init__(self, node, opts):
        aiDetector.__init__(self, node, opts)
        RelativeTimeCapability.__init__(self)
        self.request_pl('sonar_collisions.pipe')
        self.Keypoints_Name = self.options.KeyPoints_Name
        self.last_keypoints = None
        self.nearby_detected = 0
        self.last_time = 0
    
    def checkStaticCollision(self, keypoints):
        nearby = 0
        for kp in keypoints:
            if kp.x * self.options.Metres_Per_Px < self.options.Too_Close_Static:
                nearby += 1
        if nearby == 0 and self.nearby_detected > 0:
            self.nearby_detected /= 1.2
        else:
            self.nearby_detected += nearby
        msg = 'nearby sonar responses: %s/%s' % (self.nearby_detected, self.options.Too_Close_Threshold)
        if self.nearby_detected > self.options.Too_Close_Threshold:
            info(msg)
            self.detected = True
        else:
            debug(msg)
    
    def checkDynamicCollisions(self, kps_last, kps_now, dt):
        # we're dealing with a small number of keypoints so this is okay!
        kp_vectors = []
        m_per_px = self.options.Metres_Per_Px
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
            if vx <= 0:
                debug('kp moving away: vx=%s %s' % (vx*m_per_px, kp))
                continue
            # 0-----------------0---->x
            # |           o   |       60
            # |\        /     |
            # ||      o       |
            # ||vehicle width - 50
            # ||   .          |
            # |/ .            |
            # |.              |
            #.y               - 100
            #
            time_to_collision = kp.pt.x / vx
            if time_to_collision > min((3*dt,1.0)):
                debug('kp moving slowly: vx=%s: %s' % (vx*m_per_px, kp))
                continue
            y_pos_at_collision = kp.pt.y + vy * time_to_collision 
            vehicle_width_in_px = self.options.Dynamic_Vehicle_Width / m_per_px
            vehicle_y_min = 50 - 0.5*vehicle_width_in_px
            vehicle_y_max = 50 + 0.5*vehicle_width_in_px
            if y_pos_at_collision < vehicle_y_min:
                debug('kp missing left(?): %s' % kp)
                continue
            if y_pos_at_collision > vehicle_y_max:
                debug('kp missing right(?): %s' % kp)
                continue
            

    def onKeyPointsMessage(self, m):
        if m.name != self.Keypoints_Name:
            return
        self.checkStaticCollision(m.keypoints)
        if self.last_keypoints is None or not len(self.last_keypoints):
            self.last_keypoints = m.keypoints
        else:
            now = self.relativeTime()
            self.checkDynamicCollisions(self.last_keypoints, m.keypoints, now - self.last_time)
            self.last_time = now
            

