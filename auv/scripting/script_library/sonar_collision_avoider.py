import AI
from cauv.debug import debug, info, warning, error
from cauv import messaging

from collections import deque
import time, Queue


class SonarCollisionAvoider(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.points_name = 'avoid_collision'
            self.monitor_fraction = 0.25
            self.reverse_to_distance = 3.0
            self.reverse_time = 2 #seconds
            self.use_time = True
            self.use_distance = True
            self.pipeline = 'sonar_collisions_2'
            
    def __init__(self, *arg, **kwargs):
        AI.Script.__init__(self, *arg, **kwargs)
        self.messages = Queue.Queue()
        self.node.subMessage(messaging.PointsMessage())
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
        while (self.start_time+self.options.reverse_time-time.time()>0 and self.options.use_time) or \
              (self.mean_distance_closest<self.options.reverse_to_distance and self.options.use_distance):      
            self.auv.prop(-127)
            try:
                cols = map(lambda x: x.y, self.messages.get(block=True, timeout=0.5))
            except Queue.Empty:
                continue
            cols.sort()
            #check minimum distance
            #take closest points
            monitor_number = int(round(self.options.monitor_fraction*len(cols)))
            cols = cols[:monitor_number]
            #calculate average distance
            self.mean_distance_closest = sum(cols)/float(len(cols))
            debug("Mean %f" %(self.mean_distance_closest))
            time.sleep(0.5)
        self.auv.prop(0)
        

Script = SonarCollisionAvoider

if __name__ == "__main__":
    SonarCollisionAvoider.entry()
