from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, info, warning, error
from cauv import messaging

from collections import deque
import time, Queue

class scriptOptions(aiScriptOptions):
    pipeline = 'sonar_collisions_2'
    points_name = 'avoid_collision'
    monitor_fraction = 0.25
    reverse_to_distance = 3.0
    emergency_reverse_time = 2 #seconds
    use_time = True
    use_distance = True
    class Meta:
        dynamic = ['monitor_fraction', 'reverse_to_distance', 'emergency_reverse_time',
                   'points_name', 'use_distance', 'use_time']

class script(aiScript):
    debug_values = ['start_time', 'mean_distance_closest']
    def __init__(self, *arg, **kwargs):
        aiScript.__init__(self, *arg, **kwargs)
        self.messages = Queue.Queue()
        self.node.subMessage(messaging.PointsMessage())
        self.request_pl(self.options.pipeline)
        self.start_time = time.time()
        self.mean_distance_closest = 0
        
    def onPointsMessage(self, m):
        if m.name != self.options.points_name:
            return
        if len(m.points)*self.options.monitor_fraction<0.5:
            debug('Not enough points in image')
            return
        self.messages.put(m.points)
        
    def run(self):
        while (time.time()-start_time>0 or not self.options.use_time) and \
              (self.mean_distance_closest>self.options.reverse_to_distance or not self.options.use_distance):
            try:
                cols = map(lambda x: x.y, self.messages.get(block=True, timeout=0.5))
            except Queue.Empty:
                continue
            cols.sort()
            #check minimum distance
            self.auv.prop(-127)
            #take closest points
            monitor_number = int(round(self.options.monitor_fraction*len(cols)))
            cols = cols[:monitor_number]
            #calculate average distance
            self.mean_distance_closest = sum(cols)/float(len(cols))
            debug("Mean %f" %(self.mean_distance_closest))