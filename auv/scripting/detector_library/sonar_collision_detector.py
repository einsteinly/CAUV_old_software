from AI_classes import aiDetector, aiDetectorOptions
from cauv.debug import debug, info, warning, error
from cauv import messaging

from collections import deque
import time, Queue

class detectorOptions(aiDetectorOptions):
    pipeline = 'sonar_collisions_2'
    points_name = 'avoid_collision'
    monitor_fraction = 0.25
    min_distance = 0.5 #any values below this constitute and emergency
    distance_factor = 25.0
    average_count_size = 5
    class Meta:
        dynamic = ['monitor_fraction', 'min_distance', 'speed_factor',
                    'distance_factor']

class detector(aiDetector):
    debug_values = ['last_min','last_mean','speed_limit', 'detected']
    def __init__(self, *arg, **kwargs):
        aiDetector.__init__(self, *arg, **kwargs)
        self.node.subMessage(messaging.PointsMessage())
        self.request_pl(self.options.pipeline)
        self.last_min = None
        self.last_mean = None
        self.speed_limit = None
        self.messages = Queue.Queue()
        self.means = deque(maxlen=self.options.average_count_size)
        #put some initial data in to avoid errors
        self.means.append(self.options.max_distance)
        self.times.append(time.time())
        
    def onPointsMessage(self, m):
        if m.name != self.options.points_name:
            return
        if len(m.points)*self.options.monitor_fraction<0.5:
            debug('Not enough points in image')
            return
        self.messages.put(m.points)
        
    def process(self):
        while True:
            try:
                cols = map(lambda x: x.y, self.messages.get(block=True, timeout=0.5))
            except Queue.Empty:
                continue
            cols.sort()
            #check minimum distance
            self.last_min = cols[0]
            if cols[0] < self.options.min_distance:
                #too close, reverse
                self.detected = True
                return
            #take closest points
            monitor_number = int(round(self.options.monitor_fraction*len(cols)))
            cols = cols[:monitor_number]
            #calculate average distance
            self.last_mean = sum(cols)/float(len(cols))
            #if mean is to far away, no limit
            speed_limit = self.options.distance_factor*self.last_mean#factor*distance from object
            self.speed_limit = int(round(speed_limit))
            self.ai.auv_control.limit_prop(self.speed_limit)
            debug("minimum distance %f, mean %f, setting speed limit to %d" %(cols[0],self.last_mean,self.speed_limit,))