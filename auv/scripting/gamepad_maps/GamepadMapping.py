#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#


import cauv.messaging as messaging

from cauv.debug import debug, error, warning, info

class GamepadMapping(messaging.MessageObserver):
    def __init__(self, auv):
        messaging.MessageObserver.__init__(self)
        self.auv = auv
        self.bearing = 0
        self.bearing_state = False
        self.pitch = 0
        self.pitch_state = False
        self.depth = 0
        self.depth_state = False
    
    def buttonPressed(self, button):
        info( 'Button pressed %i' % (button) )
    
    def buttonReleased(self, button):
        info( 'Button released %i' % (button) )
    
    def handleAxis(self, axisId, value):
        info( 'Axis %i reads %f' % (axisId, value) )
    
    def axisNeutralValue(self, axisId):
        return 0.0
    
    def onBearingAutopilotEnabledMessage(self, m):
        debug('Received external bearing autopilot message.')
        self.bearing = m.target
        self.bearing_state = m.enabled
    
    def onPitchAutopilotEnabledMessage(self, m):
        debug('Received external pitch autopilot message.')
        self.pitch = m.target
        self.pitch_state = m.enabled
    
    def onDepthAutopilotEnabledMessage(self, m):
        debug('Received external depth autopilot message.')
        self.depth = m.target
        self.depth_state = m.enabled
        self.auv.depth_disabled = not m.enabled