#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node

import aiTypes

import cPickle as pickle
import time

class Effects:
    (bearing,
     depth,
     pitch,
     prop,
     strafe) = range(1, 6)

class DummyAUV:
    def __init__(self):
        self.effects = []
    def bearing(self, bearing):
        if not Effects.bearing in self.effects:
            self.effects.append(Effects.bearing)
    def depth(self, depth):
        if not Effects.depth in self.effects:
            self.effects.append(Effects.depth)
    def pitch(self, pitch):
        if not Effects.pitch in self.effects:
            self.effects.append(Effects.pitch)
    def strafe(self, speed):
        if not Effects.strafe in self.effects:
            self.effects.append(Effects.strafe)
    def prop(self, speed):
        if not Effects.prop in self.effects:
            self.effects.append(Effects.prop)
    def stop(self):
        self.bearing(0)
        self.depth(0)
        self.pitch(0)
        self.strafe(0)
        self.prop(0)

class TestAUV:
    def bearing(self, bearing):
        print 'bearing', bearing
    def depth(self, depth):
        print 'depth', depth
    def pitch(self, pitch):
        print 'pitch', pitch
    def strafe(self, speed):
        print 'strafe', speed
    def prop(self, speed):
        print 'speed', speed
    def stop(self):
        print 'STOP'

class DemandWrap:
    def __init__(self, demand):
        self.demand = demand
        self.dummy = DummyAUV()
        demand.execute(self.dummy)
        demand.cleanup(self.dummy)

class Controller(msg.AIMessageObserver):
    def __init__(self, auv):
        msg.AIMessageObserver.__init__(self)
        self.auv = auv
        #self.executing_demands = {} # priority (int) : Demand
        self.current_demand = None

    def onAIMessage(self, m):
        received = pickle.loads(m.msg)
        if isinstance(received, aiTypes.Demand):
            print 'received demand:', received
            self.addDemand(received)
        else:
            print 'unknown ai message:', received
    
    def addDemand(self, demand):
        # TODO: be sensible about demand conflicts, etc
        
        # TEMP: ONLY ONE OBJCTIVE IS POSSIBLE:
        # new demands from the same source override old ones
        if self.current_demand is None or \
           self.current_demand.source == demand.source or \
           self.current_demand.priority < demand.priority:
            print 'executing new demand'
            self.current_demand.cleanup(self.auv)
            self.current_demand.cleanup(TestAUV())
            self.current_demand = demand
            self.current_demand.execute(self.auv)
            self.current_demand.cleanup(TestAUV())
            

if __name__ == '__main__':
    node = cauv.node.Node('py-auto')
    auv = control.AUV(node)

    controller = Controller(auv)

    while True:
        time.sleep(1000)

