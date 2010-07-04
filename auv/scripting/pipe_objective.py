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

class PipeFollowCompleteDemand(aiTypes.Demand):
    def __init__(self):
        aiType.Demand.__init__(self)
        self.priority = 1
    def execute(self, auv):
        auv.depth(0)
        auv.prop(0)

class PipeFollowObjective(msg.BufferedMessageObserver):
    def __init__(self, node):
        msg.BufferedMessageObserver.__init__(self)
        self.__node = node
        self.__node.addObserver(self)
        self.__node.join("processing")
        self.completed = threading.Condition()
    def send(self, obj):
        self.__node.send(msg.AIMessage(pickle.dumps(obj)), "ai")
    
    def onHoughLinesMessage(self, m):
        if len(m.lines) == 0:
            print 'no lines!'
            return
        # TODO: aggregation / actual selection of best line
        best = m.lines[0]
        d = PipeFollowDemand()
        d.bearing = best.angle
        d.strafe = 50 * (best.centre.x - 0.5)
        d.prop = 50
        d.depth = 2.5
        self.send(d)

    #TODO: end of pipe
    #      turn around and follow the pipe the other way

    def run(self):
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
    



    
