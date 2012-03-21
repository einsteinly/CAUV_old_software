from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, info, warning, error

from collections import deque
import time, Queue

class scriptOptions(aiScriptOptions):
    pipeline = 'sonar_collisions_2'
    points_name = 'avoid_collision'
    monitor_fraction = 0.25
    max_distance = 5.0
    min_distance = 0.5 #any values below this constitute and emergency
    speed_fraction = 10.0
    emergency_reverse_time = 2 #seconds
    average_count_size = 5
    class Meta:
        dynamic = ['monitor_fraction', 'max_distance', 'min_distance', 'speed_fraction',
                    'emergency_reverse_time']

class script(aiScript):
    debug_values = ['mean_speed','last_min','last_mean','last_time','speed_limit', 'emergency_reversing']
    def __init__(self, *arg, **kwargs):
        aiScript.__init__(self, *arg, **kwargs)
        self.mean_speed = None
        self.last_min = None
        self.last_mean = None
        self.last_time = None
        self.speed_limit = None
        self.emergency_reversing = 0
        self.messages = Queue.Queue()
        self.means = deque(maxlen=self.options.average_count_size)
        self.times = deque(maxlen=self.options.average_count_size)
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
    def run(self):
        self.node.join('processing')
        self.request_pl(self.options.pipeline)
        while True:
            if self.in_control.is_set() and self.emergency_reversing < time.time()-self.options.emergency_reverse_time:
                self.auv.prop(0)
                self.drop_control()
            try:
                cols = map(lambda x: x.y, self.messages.get(block=True, timeout=0.5))
            except Queue.Empty:
                continue
            cols.sort()
            #check minimum distance
            self.last_min = cols[0]
            if cols[0] < self.options.min_distance:
                #too close, reverse
                debug("Emergency reversing")
                if not self.in_control.is_set():
                    self.request_control_and_wait(0.5)
                self.auv.prop(-127)
                self.emergency_reversing = time.time()
            #take closest points
            monitor_number = int(round(self.options.monitor_fraction*len(cols)))
            cols = cols[:monitor_number]
            #calculate average distance
            self.last_mean = sum(cols)/float(len(cols))
            self.last_time = time.time()
            debug("minimum distance %f, mean %f" %(cols[0],self.last_mean))
            self.times.append(self.last_time)
            #if mean is to far away, no limit
            if self.last_mean > self.options.max_distance:
                self.means.append(self.options.max_distance)
            else:
                self.means.append(self.last_mean)
            self.mean_speed = sum([(self.means[i+1]-self.means[i])/float(self.times[i+1]-self.times[i]) for i in range(len(self.means)-1)])/(len(self.means)-1)
            speed_limit = max(0,self.options.speed_fraction*self.last_mean-self.mean_speed)#factor*distance from object-speed travelling at
            self.speed_limit = int(round(speed_limit))
            self.ai.auv_control.limit_prop(self.speed_limit)
            debug("Setting speed limit to %d" %(self.speed_limit,))