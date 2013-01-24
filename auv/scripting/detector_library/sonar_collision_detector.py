#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import time
from collections import deque

from cauv.debug import debug, info, warning, error
from cauv import messaging

from AI.base.detector import aiDetector, aiDetectorOptions

class detectorOptions(aiDetectorOptions):
    points_name = 'avoid_collision'
    monitor_fraction = 0.25
    min_distance = 0.5 #any values below this constitute and emergency
    distance_factor = 25.0
    average_count_size = 5
    class Meta:
        dynamic = ['monitor_fraction', 'min_distance', 'speed_factor',
                    'distance_factor']
        pipelines = ['sonar_collisions_2']

class detector(aiDetector):
    debug_values = ['last_min','last_mean','speed_limit', 'detected']
    def __init__(self, *arg, **kwargs):
        aiDetector.__init__(self, *arg, **kwargs)
        self.node.subMessage(messaging.PointsMessage())
        self.last_min = None
        self.last_mean = None
        self.speed_limit = None
        self.messages = deque()
        
    def onPointsMessage(self, m):
        if m.name != self.options.points_name:
            return
        if len(m.points)*self.options.monitor_fraction<0.5:
            debug('Not enough points in image')
            return
        self.messages.append(m.points)
        
    def process(self):
        try:
            cols = map(lambda x: x.y, self.messages.pop())
        except IndexError:
            #no elements to process
            return
        cols.sort()
        #check minimum distance
        self.last_min = cols[0]
        if cols[0] < self.options.min_distance:
            #too close, reverse
            self.detected = True
            return
        else:
            self.detected = False
        #take closest points
        monitor_number = int(round(self.options.monitor_fraction*len(cols)))
        cols = cols[:monitor_number]
        #calculate average distance
        self.last_mean = sum(cols)/float(len(cols))
        #if mean is to far away, no limit
        speed_limit = self.options.distance_factor*self.last_mean#factor*distance from object
        self.speed_limit = int(round(speed_limit))
        #self.ai.auv_control.limit_prop(self.speed_limit)
        debug("minimum distance %f, mean %f, setting speed limit to %d" %(cols[0],self.last_mean,self.speed_limit,))
        self.messages.clear()
