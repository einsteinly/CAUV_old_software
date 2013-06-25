from cauv.node import Node
import cauv.messaging as msg

import pygame

pygame.init()
node=Node('test')

class test(msg.MessageObserver):
    def __init__(self):
        super(test, self).__init__()
        self.window = pygame.display.set_mode((640,480))
        node.subMessage(msg.RelativePositionMessage())
        node.addObserver(self)
    def onRelativePositionMessage(self, m):
        colour = (255,0,0) if m.object=="NorthWall" else ((0,255,0) if m.object=="BackWall" else ((0,0,255) if m.object=="SouthWall" else (255,255,255)))
        pygame.draw.line(self.window, colour, (320,240), (320+4*m.position.value.east,240-4*m.position.value.north))
        pygame.display.flip()
    def reset(self):
	self.window.fill((0,0,0))

sc = test()
while True:
    raw_input("Press enter to reset")
    sc.reset()
