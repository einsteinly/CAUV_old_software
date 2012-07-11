#!/usr/bin/env python2.7
from cauv.debug import debug, warning, error, info
from cauv import control

import cauv.messaging as msg

import time
import math
import argparse
import utils.event as event
from math import radians

from AI_classes import aiProcess, external_function
from utils.coordinates import LLACoord, NorthEastDepthCoord, River_Cam_Datum
from simulator import redherring_model

#need these to modify simulator
from utils.quaternion import Quaternion

class motorEstimator(redherring_model.Model):
    """
    Estimation based on motor commands, by running the simulator and updating with real bearing/detph info
    """
    def __init__(self, node):
        redherring_model.Model.__init__(self, node)
        self.node.subMessage(msg.TelemetryMessage())
        self.last_position = NorthEastDepthCoord(self.displacement[1],self.displacement[0],-self.displacement[2])
        self.start()
    def get_relative_position(self):
        #according to simulator (base_model.py) displacement is east, north, altitude
        with self.update_lock:
            position = NorthEastDepthCoord(self.displacement[1],self.displacement[0],-self.displacement[2])
        rel_pos = position - self.last_position
        self.last_position = position
        return rel_pos
    def reset(self):
        with self.update_lock:
            self.last_position = NorthEastDepthCoord(self.displacement[1],self.displacement[0],-self.displacement[2])
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

class locationFilter(msg.MessageObserver):
    """
    Combines data from various estimators to give an overall estimate of location.
    """
    def __init__(self, node):
        msg.MessageObserver.__init__(self)
        self.last_known_location = River_Cam_Datum #use this as a default, should really be set by gps
        self.last_known_time = time.time()
        self.relative_estimators = [rel_est(node) for rel_est in relative_estimators]
        self.node = node
        node.subMessage(msg.LocationMessage())
        node.addObserver(self)
    def onLocationMessage(self, msg):
        self.last_known_location = msg.location
        self.last_known_time = time.time()
        for rel_est in self.relative_estimators:
            rel_est.reset()
    def update(self):
        """
        update location based on data from estimators
        """
        estimates = [rel_est.get_relative_position() for rel_est in self.relative_estimators]
        try:
            estimated_movement = NorthEastDepthCoord(*[sum([getattr(est, d) for est in estimates]) for d in ('north', 'east', 'depth')])/len(estimates)
        except ZeroDivisionError:
            warning('No location data available, assuming not moving')
            estimated_movement = 0
        self.last_known_location += estimated_movement
        self.last_known_time=time.time()
    def get_estimate(self):
        self.update()
        return self.last_known_location

class aiLocation(aiProcess):
    def __init__(self, opts):
        aiProcess.__init__(self, 'location', opts.manager_id)
        self.location_filter = locationFilter(self.node)
        
        #start receiving messages
        self._register()

    #set via options
    @event.repeat_event(None)
    def update(self):
        last_known_location = self.location_filter.get_estimate()
        self.ai.task_manager.broadcast_position(last_known_location)

    def die(self):
        for rel_est in self.location_filter.relative_estimators:
            rel_est.stop()
        aiProcess.die(self)
    
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-t', '--update_period', dest='update_period', type=int, default=0.5, help='location updating frequency')
    p.add_argument('-M', '--manager_id', dest='manager_id', default='', action='store', help="id of relevent ai manager")
    
    opts, args = p.parse_known_args()
    
    sc = aiLocation(opts)
    sc.update.options.delay = opts.update_period
    sc.update()
    try:
        sc.run()
    finally:
        sc.node.stop()
        sc.die()
