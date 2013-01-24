#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

# listens for timeout messages an counts down when the other end is silent

import cauv.messaging as msg

import cauv
import cauv.node
from cauv.debug import debug, error, warning, info

import time
import argparse
import os


class PenultimateResort(msg.MessageObserver):
    def __init__(self, node):
        msg.MessageObserver.__init__(self)
        self.__node = node
        node.join("gui")
        
        self.lastTimeoutSet = 0
        self.lastAliveTime = 0
        
        node.addObserver(self)
    
    def onTimedOut(self):
        warning('DIE control, DIE!\n')
        node.send(msg.ProcessControlMessage(msg.ProcessCommand.Stop, "control", "*", []))
        node.send(msg.ProcessControlMessage(msg.ProcessCommand.Stop, "mcb_bridge", "*", []))
        os.system('killall -s 9 control')
        os.system('killall -s 9 mcb_bridge')
    
    def onRemoteControlsAliveMessage(self, m):
        self.lastAliveTime = time.time()
        info("remote controls alive! %d " % self.lastTimeoutSet)
    
    def onSetPenultimateResortTimeoutMessage(self, m):
        self.lastAliveTime = time.time()
        self.lastTimeoutSet = m.timeout
        info("new timeout set: %d " % self.lastTimeoutSet)
    
    def run(self):
        while True:
            time.sleep(1) # 1 second between checks
            if not self.lastTimeoutSet == 0:
                if time.time() - self.lastAliveTime > self.lastTimeoutSet:
                    self.onTimedOut()
            timeUntilOut = self.lastTimeoutSet - (time.time() - self.lastAliveTime)
            bf = msg.BoundedFloat(timeUntilOut, 0, self.lastTimeoutSet, msg.BoundedFloatType.Clamps)
            self.__node.send(msg.PenultimateResortTimeoutMessage(bf))
            info("sent status update") 
    
   
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Stop control if connection to gamepad_server is lost')
    parser.add_argument('-t', '--timeout', type=int, help='Timeout in seconds')
    opts, args = parser.parse_known_args()
    
    node = cauv.node.Node('py-penresort',args)
    try:
        d = PenultimateResort(node)
        if opts.timeout:
            d.lastAliveTime = time.time()
            d.lastTimeoutSet = opts.timeout
        d.run()
    finally:
        node.stop()
