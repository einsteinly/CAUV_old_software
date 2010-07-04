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
max_turns = 4

class PipeFollowDemand(aiTypes.Demand):
    def __init__(self, prop = None, strafe = None, bearing = None):
        aiType.Demand.__init__(self)
        self.priority = 0
        self.source = 'PFD'
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
        self.source = 'PFD'
        self.priority = 1
        self.bearing = bearing
    def execute(self, auv):
        auv.prop(0)
        auv.bearing(self.bearing)
    def cleanup(self, auv):
        pass

class PipeFollowCompleteDemand(aiTypes.Demand):
    def __init__(self):
        aiType.Demand.__init__(self)
        self.source = 'PFD'        
        self.priority = 2
    def execute(self, auv):
        auv.depth(0)
        auv.prop(0)

def angleDiff(a,b):
    d = mod(a-b, 360)
    while d <= -180:
        d = d + 360
    while d > 180:
        d = d - 360
    return d

class PipeFollowObjective(msg.BufferedMessageObserver):
    def __init__(self, node):
        msg.BufferedMessageObserver.__init__(self)
        self.__node = node
        self.__node.addObserver(self)
        self.__node.join("processing")
        self.lock = threading.Lock()
        self.bearing = 0

    def send(self, obj):
        self.__node.send(msg.AIMessage(pickle.dumps(obj)), "ai")
    
    def lineBearing(self, l):
        b = l.angle
        if b <= -90:
            b = b + 180
        if b > 90:
            b = b - 180
        return self.bearing + b

    def onTelemetryMessage(self, m):
        self.bearing = m.orientation.yaw

    def onHoughLinesMessage(self, m):
        if len(m.lines) == 0:
            print 'no lines!'
            return
        self.lock.acquire()

        best = None
        if len(m.lines) > 1:
            bestHeadingDiff = 45 # Don't want a sudden sharp turn
            if self.previousPipeHeading == None:
                self.previousPipeHeading = self.bearing
                bestHeadingDiff = 360 # fuck it, accept all lines
            for line in m.lines:
                bearing = lineBearing(line)
                diff = abs(angleDiff(bearing, self.previousPipeHeading))
                if diff < bestHeadingDiff:
                    bestHeadingDiff = diff
                    best = line
        else:
            best = m.lines[0]
        
        if best == None:
            print 'motherfucker, no good lines'
            return

        self.previousPipeHeading = lineBearing(best)

        d = PipeFollowDemand()
        d.bearing = self.previousPipeHeading
        d.strafe = 50 * (best.centre.x - 0.5)
        d.depth = follow_depth
        d.prop = follow_prop
        self.last_line_time = time.time()
        self.send(d)
        self.lock.release()        



    #TODO: end of pipe
    #      turn around and follow the pipe the other way

    def run(self):
        turns = 0
        while True:
            time.sleep(0.2)
            self.lock.acquire()            
            if last_line_time is not None and time.time() - self.last_line_time > reverse_delay:
                print 'not seen pipe for a while: turning around'
                self.send(TurnAround(self.bearing + 180))
                turns += 1
                time.sleep(8)
            self.lock.release()
            if turns > max_turns:
                print 'turned too many times: stopping'
                self.send(PipeFollowCompleteDemand())
                time.sleep(5)
                break

if __name__ == '__main__':
    node = cauv.node.Node('py-pipe')    
    pfo = PipeFollowObjective(node)

    try:
        pfo.run()
    except Exception, e:
        traceback.print_exc()

    print 'pipe-following objective complete'
    



    
