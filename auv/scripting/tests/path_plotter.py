#!/usr/bin/env python2.7
from cauv.node import Node
import cauv.messaging as msg

from math import radians, cos, sin

import pygame

pygame.init()
node=Node('test')

angle = int(raw_input("Enter bay angle: "))

class test(msg.MessageObserver):
    def __init__(self):
        super(test, self).__init__()
        self.window = pygame.display.set_mode((640,480))
        node.subMessage(msg.RelativePositionMessage())
        node.addObserver(self)
        pygame.draw.line(self.window, (255,255,0), (320,240), (320-400*cos(radians(angle)),240+400*sin(radians(angle))))
        pygame.draw.line(self.window, (255,255,0), (320,240), (320-200*cos(radians(angle+90)),240+200*sin(radians(angle+90))))
        pygame.draw.line(self.window, (255,255,0), (320-200*cos(radians(angle+90)),240+200*sin(radians(angle+90))), (320-200*cos(radians(angle+90))-400*cos(radians(angle)),240+200*sin(radians(angle+90))+400*sin(radians(angle))))
        pygame.display.flip()
    def onRelativePositionMessage(self, m):
        if m.object != "NECorner":
            return
        pygame.draw.rect(self.window, (255, 255, 255), ((int(320-m.position.value.east*4),int(240+m.position.value.north*4)), (1,1)), 1)
        pygame.display.flip()
    def reset(self):
        self.window.fill((0,0,0))
        pygame.draw.line(self.window, (255,255,0), (320,240), (320-400*cos(radians(angle)),240+400*sin(radians(angle))))
        pygame.draw.line(self.window, (255,255,0), (320,240), (320-200*cos(radians(angle+90)),240+200*sin(radians(angle+90))))
        pygame.draw.line(self.window, (255,255,0), (320-200*cos(radians(angle+90)),240+200*sin(radians(angle+90))), (320-200*cos(radians(angle+90))-400*cos(radians(angle)),240+200*sin(radians(angle+90))+400*sin(radians(angle))))
        pygame.display.flip()

sc = test()
while True:
    raw_input("Press enter to reset")
    sc.reset()
