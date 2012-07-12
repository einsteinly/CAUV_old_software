#!/usr/bin/env python2.7

import cauv.messaging as messaging

import cauv
import cauv.control as control
import cauv.messaging as msg
import cauv.node
from cauv.debug import debug, error, warning, info

import time
import math
import argparse
import pygame

from gamepad_maps.DefaultMapping import ConcreteGamepadMapping
from gamepad_maps.GamepadMapping import GamepadMapping


class GamepadServer(messaging.MessageObserver):
    def __init__(self, node, rate, deadband):
        messaging.MessageObserver.__init__(self)
        self.__node = node
        node.join("control")
        node.join("gui")
        self.lastButtonValues = {}
        self.lastAxisValues = {}
        self.rate = rate
        self.deadband = deadband
        self.hasGamepad = False
        info("Gamepad server starting up...")

        node.addObserver(self)
        
        node.send(msg.SetPenultimateResortTimeoutMessage(10))
        
        pygame.init()
        if pygame.joystick.get_count() == 0:
            error ("No gamepads detected")
            raise IOError()
        else: 
            self.hasGamepad = True
        
        if self.hasGamepad:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            info( "Initialized Joystick : %s" % self.joystick.get_name() )

            for i in range(0, self.joystick.get_numbuttons()):
                self.lastButtonValues[i] = 0
            for i in range(0, self.joystick.get_numaxes()):
                self.lastAxisValues[i] = 0.00
    
    def gamepadName(self):
        return self.joystick.get_name()


    def handleEvents(self):
        if not self.hasGamepad:
            return
        pygame.event.pump()
        for i in range(0, self.joystick.get_numaxes()):
            currentValue = self.joystick.get_axis(i)
            neutralValue = self.mapping.axisNeutralValue(i)
            
            if (math.fabs(currentValue - neutralValue)) > self.deadband:
                self.mapping.handleAxis(i, currentValue )
                self.lastAxisValues[i] = currentValue
            else:
                if self.lastAxisValues[i] != neutralValue:
                    self.mapping.handleAxis(i, neutralValue )
                self.lastAxisValues[i] = neutralValue
        
        for i in range(0, self.joystick.get_numbuttons()):
            currentValue = self.joystick.get_button(i)
            if self.lastButtonValues[i] != currentValue:
                if currentValue:
                    self.mapping.buttonPressed(i)
                else:
                    self.mapping.buttonReleased(i)
            self.lastButtonValues[i] = currentValue
            
    def run(self):
        try:
            iterations = 1/self.rate # send an alive message once per second
            count = 0
            while True:
                self.handleEvents()
                time.sleep(self.rate)
                if count > iterations:
                    info("sending alive message")
                    self.__node.send(msg.RemoteControlsAliveMessage())
                    count = 0
                count = count + 1
        
                
        except KeyboardInterrupt:
            if self.hasGamepad:
                self.joystick.quit()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Provide gampepad input to control the AUV", add_help = True)
    parser.add_argument("--mapping", "-m", help="force mapping file to use")
    parser.add_argument("--rate", "-r", help="maximum poll rate (seconds)", default=0.05)
    parser.add_argument("--deadband", "-d", help="axis deadband", default=0.3)
    parser.add_argument("--debug", "-D", help="show buttons pressed (no output)", action="store_true", default = False)
    parser.add_argument("--controls", "-c", help="show list of controls", action="store_true", default = False)
    args, unknown = parser.parse_known_args()
    
    info("Node starting up...")

    node = cauv.node.Node('py-gmpd')
    try:
        auv = control.AUV(node)
        d = GamepadServer(node, args.rate, args.deadband)
        
        if d.hasGamepad:
            mapFile = ("gamepad_maps.%s" % d.gamepadName().replace(' ',''))
            if args.mapping:
                mapFile = ("gamepad_maps.%s" % args.mapping)
            module = __import__(mapFile, fromlist=['gamepad_maps'])
            info('Imported mapping module '+str(module))
            if args.debug:
                d.mapping = GamepadMapping(auv)
            else:
                d.mapping = ConcreteGamepadMapping(auv, module.XBoxAxes, module.XBoxButtons)
            if args.controls:
                try:
                    print d.mapping.controls()
                except AttributeError:
                    print "Mapping did not provide controls listing"
            else:
                node.addObserver(d.mapping)
        d.run()
    except IOError:
        info("Stopping due to IO Error")
    finally:
        node.stop()
