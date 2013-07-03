import cauv.control
import cauv.messaging as messaging
import utils.control
from utils.event import EventLoop, event_func, repeat_event
from cauv.debug import debug, info, error, warning

import time
import threading
from math import atan2, sqrt, cos, sin, degrees

C2D = messaging.CartesianPosition2D
                
class RelativeLocationMover(EventLoop):
    """
    Event loop for moving the vehicle to a relative location
    """
    def __init__(self, auv, target, error, search_timeout = 0.5, search_angle = 5, search_hold = 2,
                    controller_p = utils.control.PIDController((20,0,0)),
                    controller_s = utils.control.PIDController((20,0,0))):
        EventLoop.__init__(self)
        self.target = target
        self.target_error = error
        self.search_timeout = search_timeout
        self.search_angle = search_angle
        self.search_hold = search_hold
        #inject position handler
        self.auv = auv
        self.auv.RelativePositionMessageHandlers.append(self.onRelativePositionMessage)
        #control loops
        self.controller_p = controller_p
        self.controller_s = controller_s
        #decision values for when and where to search
        self.start_time = time.time()
        self.last_message_time = time.time()
        self.searching = False
        self.search_bearing = None
        self.search_direction = 1
        self.is_in_range = threading.Event()
        
    
    @event_func
    def onRelativePositionMessage(self, m):
        if m.origin != "AUV":
            return
        if m.object != "NECorner":
            return
        #turn off searching
        self.searching = False
        #face towards NE corner
        info("NECorner @ "+ str(m.position.value))
        bearing = atan2(m.position.value.east, m.position.value.north)
        self.auv.bearingAndWait(degrees(bearing))
        
        #calculate error
        prop_error = (m.position.value.north+self.target[0])*cos(bearing)+sin(bearing)*(m.position.value.east+self.target[1])
        strafe_error = (m.position.value.north+self.target[0])*sin(bearing)-cos(bearing)*(m.position.value.east+self.target[1])
        
        if abs(prop_error) < self.target_error and abs(strafe_error) < self.target_error:
            self.is_in_range.set()
        
        prop=int(max(min(self.controller_p.update(prop_error), 127), -127))
        strafe=int(max(min(self.controller_s.update(strafe_error), 127), -127))
        info("Prop %f, Strafe %f" %(prop, strafe))
        self.auv.prop(prop)
        self.auv.strafe(strafe)
        self.last_message_time = time.time()
    
    @repeat_event(autostart = True, time_period = 0.1)
    def search(self):
        if not self.searching:
            #if not within timeout, escape
            if time.time() - self.last_message_time < self.search_timeout:
                return
            #timed out, so intialise search
            info("Searching.")
            self.searching = True
            self.auv.prop(0)
            self.auv.strafe(0)
            #make sure searching in a suitable direction
            self.search_bearing = self.auv.current_bearing
            #TODO there should be some logic to avoid looking out to see here
        self.search_bearing += self.search_angle*self.search_direction
        self.auv.bearingAndWait(self.search_bearing)
        
    def wait(self, timeout):
        return self.is_in_range.wait(timeout)
        
    def end(self):
        self.auv.RelativePositionMessageHandlers.remove(self.onRelativePositionMessage)
        self.stop()
        #try and stop
        self.auv.stop()
        #wait for the mover to stop
        self.join()
        #make sure stoppped
        self.auv.stop()

class AUV(cauv.control.AUV):
    def __init__(self, node, **kwargs):
        super(AUV, self).__init__(node, **kwargs)
        self.RelativePositionMessageHandlers = []
        self.node.subMessage(messaging.RelativePositionMessage())
        
    def onRelativePositionMessage(self, m):
        for handler in self.RelativePositionMessageHandlers:
            handler(m)
            
    def moveToRelativeLocation(self, target, timeout = 60, error = 1):
        """
        Move to a location. Stops when either timer runs out, or within error of location.
        Returns false if the timer runs out, true otherwise.
        Timeout can be None
        """
        mover = RelativeLocationMover(self, target, error)
        try:
            mover.start()
            value = mover.wait(timeout)
        finally:
            mover.end()
        return value