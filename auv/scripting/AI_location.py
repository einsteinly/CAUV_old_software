#!/usr/bin/env python2.7
from cauv.debug import debug, warning, error, info
from cauv import control

import cauv.messaging as msg

import time
import math
import argparse
import traceback
import threading
from math import radians

from AI_classes import aiProcess, external_function
from utils.coordinates import LLACoord, NorthEastDepthCoord, River_Cam_Datum
from simulator import redherring_model

#need these to modify simulator
from utils.quaternion import Quaternion

SANITY_CUTOFF = 10.0

class aiEstimator(msg.MessageObserver):
    def __init__(self, node):
        self.node = node #dont clear up this node, as its the same node as in aiLocation

class aiRelativeEstimator(aiEstimator):
    """
    template for what a relative estimator should look like,
    relative estimators give a relative position, i.e. have accumulating error with time
    """
    def __init__(self, node):
        aiEstimator.__init__(self, node)
    def get_relative_position(self):
        """
        return the change in position since the last reading
        this needs to be a fast function, as it has to be called on all estimators at nearly the same time
        """
        raise NotImplementedError

class aiAbsoluteEstimator(aiEstimator):
    """
    template for a absolute estimator
    absolute estimators give absolute position, i.e. error is not related to running time, e.g. GPS
    """
    #this hasn't been defined properly yet since it hasnt been needed
    pass
        
class motorEstimator(redherring_model.Model, aiRelativeEstimator):
    """
    Estimation based on motor commands, by running the simulator and updating with real bearing/detph info
    """
    def __init__(self, node):
        redherring_model.Model.__init__(self, node)
        self.node.join('telemetry')
        aiRelativeEstimator.__init__(self, node)
        self.last_position = NorthEastDepthCoord(self.displacement[1],self.displacement[0],-self.displacement[2])
        self.start()
    def get_relative_position(self):
        #according to simulator (base_model.py) displacement is east, north, altitude
        with self.update_lock:
            position = NorthEastDepthCoord(self.displacement[1],self.displacement[0],-self.displacement[2])
        rel_pos = position - self.last_position
        self.last_position = position
        return rel_pos, 0.5
    #Overridden commands
    #this stops the model from communicating (and causing untold problems)
    def sendStateMessages(self):
        pass
    #update our estimated orientation and depth with actual values
    def onTelemetryMessage(self, m):
        with self.update_lock:
            self.orientation = Quaternion.fromEuler((radians(m.orientation.pitch),radians(m.orientation.roll),radians(-m.orientation.yaw)))
            #according to simulator, displacement uses altitude
            self.displacement[2]=-m.depth
        
    
relative_estimators = [motorEstimator]

class locationFilter(object):
    """
    Combines data from various estimators to give an overall estimate of location.
    """
    def __init__(self, relative_estimators=[]):
        self.last_known_location = River_Cam_Datum #use this as a default, should really be set by gps
        self.last_known_time = time.time()
        self.relative_estimators = relative_estimators
    def update(self):
        """
        update location based on data from estimators
        note doesn't use absolute estimators yet
        """
        estimates = [rel_est.get_relative_position() for rel_est in self.relative_estimators]
        total_estimate = NorthEastDepthCoord(0,0,0)
        total_weight = 0
        for estimate, weight in estimates:
            total_estimate += estimate*weight
            total_weight += weight
        try:
            estimated_movement = total_estimate/total_weight
        except ZeroDivisionError:
            warning('No location data available, assuming not moving')
            estimated_movement = 0
        self.last_known_location += estimated_movement
        self.last_known_time=time.time()
    def get_estimate(self):
        self.update()
        return self.last_known_location

class aiLocation(aiProcess):
    def __init__(self, opts, args):
        aiProcess.__init__(self, 'location')
        self.options = opts.__dict__
        self.args = args
        self.location_filter = locationFilter([rel_est(self.node) for rel_est in relative_estimators])
    def run(self):
        while True:
            time.sleep(self.options['update_period'])
            last_known_location = self.location_filter.get_estimate()
            self.ai.task_manager.broadcast_position(last_known_location)
    def die(self):
        for rel_est in self.location_filter.relative_estimators:
            rel_est.stop()
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
