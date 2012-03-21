from AI_classes import aiScript, aiScriptOptions
from cauv.debug import debug, info, warning, error

from Queue import Queue
from collections import deque
import time

class scriptOptions(aiScriptOptions):
    points_name = 'avoid_collision'
    monitor_fraction = 0.25
    max_distance = 5.0
    min_distance = 1.0 #any values below this constitute and emergency
    speed_fraction = 200.0
    emergency_reverse_time = 2 #seconds
    average_count_size = 5
    class Meta:
        dynamic = ['monitor_fraction', 'max_distance', 'min_distance', 'speed_fraction',
                    'emergency_reverse_time']

class script(aiScript):
    def __init__(self, *arg, **kwargs):
        aiScript.__init__(self, *arg, **kwargs)
        self.messages = Queue()
        self.mean_speed = 0
        self.means = deque(maxlen=self.options.average_count_size)
        self.times = deque(maxlen=self.options.average_count_size)
        #put some initial data in to avoid errors
        self.means.append(self.options.max_distance)
        self.times.append(time.time())
    def onPointsMessage(self, m):
        if m.name != self.options.points_name:
            return
        self.messages.put(m.points)
    def run(self):
        self.node.join('processing')
        while True:
            cols = map(lambda x: x.y, self.messages.get(block=True))
            cols.sort()
            #check minimum distance
            if cols[0] < self.options.min_distance:
                #too close, reverse
                debug("Emergency reversing")
                self.request_control_and_wait(0.5)
                self.auv.prop(-127)
                time.sleep(self.options.emergency_reverse_time)
                self.auv.prop(0)
                self.drop_control()
            #now calculate average distance
            monitor_number = int(round(self.options.monitor_fraction*len(cols)))
            cols = cols[:monitor_number]
            mean = sum(cols)/float(len(cols))
            debug("minimum distance %f, mean %f" %(cols[0],mean))
            self.times.append(time.time())
            #if mean is to far away, no limit
            if mean > self.options.max_distance:
                self.means.append(self.options.max_distance)
                self.ai.auv_control.limit_prop(127)
                debug("Setting no limit to prop value", 5)
            else:
                self.means.append(mean)
                speed_limit = self.options.speed_fraction*(mean-self.options.min_distance)/(sum(self.means)/float(self.times[-1]-self.times[0]))#factor*distance from object/speed travelling at
                speed_limit = int(round(speed_limit))
                self.ai.auv_control.limit_prop(speed_limit)
                debug("Setting speed limit to %d" %(speed_limit,))