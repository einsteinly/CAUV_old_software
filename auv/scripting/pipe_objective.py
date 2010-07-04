#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.node

import aiTypes

import cPickle as pickle
import time
import threading
import math
import traceback

follow_depth = 2.5
follow_prop = 100
reverse_delay = 2

class PipeFollowDemand(aiTypes.Demand):
    def __init__(self, prop = None, strafe = None, bearing = None):
        aiType.Demand.__init__(self)
        self.priority = 0
        self.prop = prop
        self.strafe = strafe
        self.bearing = bearing
        self.depth = depth
    def execute(self, auv):
        if self.prop is not None:
            auv.prop(self.prop)
        if self.strafe is not None:
            auv.strafe(self.strafe)
        if self.bearing is not None:
            auv.bearing(self.bearing)
        if self.depth is not None:
            auv.depth(self.depth)
    def cleanup(self, auv):
        pass

class TurnAround(aiTypes.Demand):
    def __init__(self, bearing):
        aiType.Demand.__init__(self)
        self.bearing = bearing
    def execute(self, auv):
        auv.prop(0)
        auv.bearing(self.bearing)
    def cleanup(self, auv):
        pass

class PipeFollowCompleteDemand(aiTypes.Demand):
    def __init__(self):
        aiType.Demand.__init__(self)
        self.priority = 1
    def execute(self, auv):
        auv.depth(0)
        auv.prop(0)

def angleDiff(a,b):
    d = mod(a-b, 360)
    while d <= -180:
        d = d + 360
    while d > 180:
        d = d - 360


class PipeFollowObjective(msg.BufferedMessageObserver):
    def __init__(self, node):
        msg.BufferedMessageObserver.__init__(self)
        self.__node = node
        self.__node.addObserver(self)
        self.__node.join("processing")
        self.completed = threading.Condition()#
        self.turning = False
    def send(self, obj):
        self.__node.send(msg.AIMessage(pickle.dumps(obj)), "ai")
    

    def onTelemetryMessage(self, m):
        self.bearing = m.orientation.yaw

    def onHoughLinesMessage(self, m):
        if len(m.lines) == 0:
            print 'no lines!'
            return
        if self.turning:
            print 'turning: ignoring lines'
            return
        
        best = m.lines[0]
        if len(m.lines) > 1:
            if self.previousPipeHeading != None:
                bestHeadingDiff = 360 # > 180
                for line in m.lines:
                    lineBearing = line.angle + self.bearing
                    diff = angleDiff(line.angle, self.previousPipeHeading)
                    if diff < bestHeadingDiff:
                        bestHeadingDiff = diff
                        best = line
        
        self.previousPipeHeading = best.angle + self.bearing

        d = PipeFollowDemand()
        d.bearing = self.previousPipeHeading
        d.strafe = 50 * (best.centre.x - 0.5)
        d.depth = follow_depth
        d.prop = follow_prop
        self.last_line_time = time.time()
        self.send(d)



    #TODO: end of pipe
    #      turn around and follow the pipe the other way

    def run(self):
        while True:
            time.sleep(200)
            if last_line_time is not None and time.time() - self.last_line_time > reverse_delay:
                print 'not seen pipe for a while: turning around'
                self.turning = True
                self.send(TurnAround(self.bearing + 180))
                time.sleep(8)
                self.turning = False
        self.completed.acquire()
        self.completed.wait()

if __name__ == '__main__':
    node = cauv.node.Node('py-pipe')    
    pfo = PipeFollowObjective(node)

    try:
        pfo.run()
    except Exception, e:
        traceback.print_exc()

    print 'pipe-following objective complete'
    



    
