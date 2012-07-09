from cauv.debug import debug, info, warning, error 
from AI_classes import aiScript, aiScriptOptions

from math import atan2

class scriptOptions(aiscriptOptions):
    Centre_Name = asv_centre
    Prop_K = 127*2
    Strafe_K = 127*2
    Pipeline = 'track_asv'
    
class script(aiScript):
    def __init__(self, *args, **kwargs):
        aiScript.__init__(self, *args, **kwargs)
        self.request_pl(self.options.Pipeline)
    def onCentreMessage(self, m):
        self.centre_on(m.x, m.y)
        
    def centre_on(x, y):
        #  |
        #__|___________x
        #  |
        #  |
        #  |     .(0.5,0.5)
        #  |
        #  |
        #  y
        #set strafe and prop for instant following
        self.auv.prop(int(self.options.Prop_K*(0.5-y))
        self.auv.strafe(int(self.options.Strafe_K*(x-0.5))
        #set bearing to try and get in line with the asv
        bearing_change = atan2(x-0.5, 0.5-y)
        self.auv.bearing(self.auv.current_bearing + bearing_change)
    
