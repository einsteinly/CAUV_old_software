#!/usr/bin/env python2.7

import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.node
from cauv.debug import debug, error, warning, info

import time
import math
import argparse
import pygame

class GamepadServer(messaging.MessageObserver):
    def __init__(self, node):
        messaging.MessageObserver.__init__(self)
        self.__node = node
        node.join("control")
        node.join("gui")
        
        info("Gamepad server starting up...")

        node.addObserver(self)
        
        pygame.init()
        if pygame.joystick.get_count() == 0:
            error ("No gamepads detected")
            return
        j = pygame.joystick.Joystick(0)
        j.init()
        info( 'Initialized Joystick : %s' % j.get_name() )
        try:
            while True:
                pygame.event.pump()
                for i in range(0, j.get_numaxes()):
                    if j.get_axis(i) != 0.00:
                        info( 'Axis %i reads %.2f' % (i, j.get_axis(i)) )
                for i in range(0, j.get_numbuttons()):
                    if j.get_button(i) != 0:
                        info( 'Button %i reads %i' % (i, j.get_button(i)) )
        except KeyboardInterrupt:
            j.quit()
   
if __name__ == '__main__':
    parser = argparse.ArgumentParser(usage='usage: %prog')
        #parser.add_argument("-m", "--mode", dest="mode", default="simple",
        #    help="integration mode: 'simple' or 'exponential' see" +
        #    "displacement_integrator.py for exponential integrator constants")
    opts, args = parser.parse_known_args()
    
    info("Node starting up...")

    node = cauv.node.Node('py-dspl', args)
    try:
        auv = control.AUV(node)
        d = GamepadServer(node)
        node.run()
    finally:
        node.stop()
