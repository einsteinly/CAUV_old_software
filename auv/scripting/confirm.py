#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from colourfinder import ColourFinder as ColFinder

import time
import traceback
import threading

class Confirmer(messaging.BufferedMessageObserver):

    def __init__(self, node, auv, name):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("processing")
        node.addObserver(self)
        self.bin = bin
        self.lock = threading.Lock()

    def confirm(): 
        try:
            print 'Emergency stop.'
            #calibrate
            auv.prop(-127)
            time.sleep(0.5)
            auv.prop(0)
            while
            #pipeline change
        except Exception:
            traceback.print_exc()
            auv.depth(0)
            auv.stop()

        print 'Complete.'
        return 0

    def onCentreMessage(self, m):
        if(m.name == self.name):
            lock.acquire()
            self.cX = m.x
            self.cY = m.y
            lock.release()

if __name__ == '__main__':
    node = cauv.node.Node('Confirm')
    print 'Setting calibration.'
    node.send(msg.DepthCalibrationMessage(
    -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    ), "control")

    auv.bearingParams(1, 0, -80, 1)
    auv.depthParams(40, 0, 0, 1)
    #auv.pitchParams(1, 0, 0, 1)

    auv.propMap(10, -10, 127, -127)
    auv.vbowMap(10, -10, 127, -127)
    auv.hbowMap(10, -10, 127, -127)
    auv.vsternMap(10, -10, 127, -127)
    auv.hsternMap(10, -10, 127, -127)
    Confimer(node, auv, 'Yellow').confirm()
    while True:
        time.sleep(5)
