from cauv.debug import debug, warning, error, info
from cauv import control

import cauv.messaging as msg

import time
import math
import argparse
import traceback
import threading

from AI_classes import aiProcess, external_function
from utils.vectormath import vec
from simulator import redherring_model

from Quaternion import Quat

SANITY_CUTOFF = 10.0

class aiEstimator(msg.MessageObserver):
    def __init__(self, node):
        self.node = node #dont clear up this node, as its the same node as in aiLocation

class aiVelocityEstimator(aiEstimator):
    def __init__(self, node):
        aiEstimator.__init__(self, node)
    def get_relative_position(self):
        pass

class aiLocationEstimator(aiEstimator):
    def __init__(self, node, id, location_filter):
        aiEstimator.__init__(self, node)
        self.id = id
        self.location_filter = location_filter
    def update(self, location, probability):
        #first sanity check
        estimated_location, estimate_probability = self.location_filter.get_estimate()
        if estimate_probability*(estimate_location-location)/probability > SANITY_CUTOFF:
            #might want to ignore in future
            return
        #then update
        self.location_filter.update(self.id, estimate, probability)
        
class motorEstimator(redherring_model.Model, aiVelocityEstimator):
    def __init__(self, node):
        redherring_model.Model.__init__(self, node)
        aiVelocityEstimator.__init__(self, node)
        self.start()
    def get_relative_position(self):
        return vec(self.displacement[0], self.displacement[1]), 0.9
    def reset(self):
        self.displacement[0]=0
        self.displacement[1]=0
    #Overridden commands
    def sendStateMessages(self):
        pass
    def onTelemetryMessage(self, m):
        self.oreintation = Quat((m.orientation.yaw,m.orientation.roll,m.orientation.pitch))
        self.displacement[3]=m.depth
        
    
velocity_estimators = [motorEstimator]
location_estimators = []

class locationFilter(object):
    def __init__(self):
        self.last_known_location = vec(0,0)
        self.last_known_time = time.time()
        self.coord_displacements = {}
        self.velocity_estimators = []
    def update(self, id, estimate, probability):
        if not id in self.coord_displacements:
            #use first estimate as 'initial' value
            self.coord_displacements[id] = estimate-self.get_estimate() #ie 0 position
            return
        #this needs to be much better
        self.last_known_location = (1-probability)*self.get_estimate()+probability*estimate
        self.last_known_time = time.time()
        for vel_est in self.velocity_estimators:
            vel_est.reset()
    def add_velocity_estimator(self, vel_est):
        self.velocity_estimators.append(vel_est)
        vel_est.reset()
    def get_estimate(self):
        total_diff = vec(0,0)
        total_prob = 0
        for vel_est in self.velocity_estimators:
            posn, probability = vel_est.get_relative_position()
            total_diff += posn*probability
            total_prob += probability
        #normalise
        total_diff /= total_prob
        return self.last_known_location+total_diff, total_prob/len(velocity_estimators)

class aiLocation(aiProcess):
    def __init__(self, opts, args):
        aiProcess.__init__(self, 'location')
        self.options = opts.__dict__
        self.args = args
        self.location_filter = locationFilter()
        self.location_estimators = []
        #initialise estimators
        for estimator_class in velocity_estimators:
            self.location_filter.add_velocity_estimator(estimator_class(self.node))
        for id, estimator_class in enumerate(location_estimators):
            self.location_estimators.append(estimator_class(self.node, id, self.location_filter))
    def run(self):
        while True:
            time.sleep(self.options['update_period'])
            estimate, probability = self.location_filter.get_estimate()
            self.ai.task_manager.broadcast_position(estimate)
    def die(self):
        for vel_est in self.location_filter.velocity_estimators:
            vel_est.stop()
        aiProcess.die(self)
    
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-t', '--update_period', dest='update_period', type=int, default=0.5, help='location updating frequency')
    
    opts, args = p.parse_known_args()
    
    sc = aiLocation(opts, args)
    try:
        sc.run()
    finally:
        sc.node.stop()
        sc.die()
