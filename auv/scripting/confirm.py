#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from colourfinder import ColourFinder as ColFinder

import time
import traceback

class Confirmer(messaging.BufferedMessageObserver):

    def __init__(self, node, auv):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("processing")
        node.addObserver(self)

    def Confirm(): 
        #node = cauv.node.Node('py-confirm')             #Create a node of the spread messaging service
        #auv = control.AUV(node)                         #Create a python object for the control of the AUV
        #detect = ColFinder(node, 14, 'Hue')             #Turn on the colour detection script

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

        try:
        print 'Emergency stop.'
        auv.prop(-127)
        time.sleep(0.5)
        auv.prop(0)
        #pipeline change

        except Exception:
        traceback.print_exc()
        auv.depth(0)
        auv.stop()
        print 'Complete.'

        return 0
    
if __name__ == '__main__':
    Search()




