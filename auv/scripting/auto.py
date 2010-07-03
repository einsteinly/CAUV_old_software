#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node

import aiTypes

import cPickle as pickle
import time


'''
# set-up calibration factors
node.send(msg.DepthCalibrationMessage(
    -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
))

auv.bearingParams(1, 0, -80, 1)
auv.depthPatams(-40, 0, 0, 1)
auv.pitchParams(1, 0, 0, 1)

auv.propMap(10, -10, 127, -127)
auv.vbowMap(10, -10, 127, -127)
auv.hbowMap(10, -10, 127, -127)
auv.vsternMap(10, -10, 127, -127)
auv.hsternMap(10, -10, 127, -127)
'''

class Effects:
    (bearing,
     depth,
     pitch,
     prop,
     strafe) = range(1, 4)

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
        if not Effects.strage in self.effects:
            self.effects.append(Effects.strafe)
    def prop(self, speed):
        if not Effects.prop in self.effects:
            self.effects.append(Effects.prop)
    def stop(self)
        self.bearing()
        self.depth()
        self.pitch()
        self.strafe()
        self.prop()

class DemandWrap:
    def __init__(self, demand):
        self.demand = demand
        self.dummy = DummyAUV()
        demand.execute(dummy)
        demand.cleanup(dummy)

class Controller(msg.AIMessageObserver):
    def __init__(self, auv):
        self.auv = auv
        msg.AIMessageObserver.__init__(self)
        #self.executing_demands = {} # priority (int) : Demand
        self.current_demand = None

    def onAIMessage(self, m):
        received = pickle.loads(m.msg)
        if isinstance(received, aiTypes.Demand):
            print 'received demand:', received
            self.addDemand(received)
        else
            print 'unknown ai message:', received
    
    def addDemand(self, demand):
        # TODO: be sensible about demand conflicts, etc
        
        # TEMP: ONLY ONE OBJCTIVE IS POSSIBLE:
        # new demands from the same source override old ones
        if self.current_demand is None or \
           self.current_demand.source == demand.source or \
           self.current_demand.priority < demand.priority:
            self.current_demand.cleanup(self.auv)
            self.current_demand = demand
            self.current_demand.execute(self.auv)
            

if __name__ == '__main__':
    node = cauv.node.Node('py-auto')
    auv = control.AUV(node)

    controller = Controller(auv)

    while True:
        time.sleep(1000)

