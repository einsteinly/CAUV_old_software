from cauv.debug import debug, info, warning, error 
import AI

from math import atan2
    
class TrackASV(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.Centre_Name = asv_centre
            self.Prop_K = 127*2
            self.Strafe_K = 127*2
            self.pipeline = 'track_asv'
        
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
        self.auv.prop(int(self.options.Prop_K*(0.5-y)))
        self.auv.strafe(int(self.options.Strafe_K*(x-0.5)))
        #set bearing to try and get in line with the asv
        bearing_change = atan2(x-0.5, 0.5-y)
        self.auv.bearing(self.auv.current_bearing + bearing_change)
        

Script = TrackASV

if __name__ == "__main__":
    TrackASV.entry()
    
