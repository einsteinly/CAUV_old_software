import messaging
import threading

class AUV(messaging.BufferedMessageObserver):
    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        node.join("control")
        node.addObserver(self)
        
        ## synchronising stuff
        self.received_state_condition = threading.Condition()
        self.received_state = None

    def send(self, msg):
        # send to control via self.__node
        self.__node.send(msg, "control")

    def stop(self):
        self.prop(0)
        self.hbow(0)
        self.vbow(0)
        self.hstern(0)
        self.vstern(0)
        self.bearing(None)
        self.pitch(None)
        self.depth(None)

    def bearing(self, timeout=3):
        self.received_state_condition.acquire()
        self.received_state = None
        self.send(messaging.StateRequestMessage())
        self.received_state_condition.wait(timeout)
        self.received_state_condition.release()
        return self.received_state.orientation

    def bearing(self, bearing):
        if bearing is not None:
            self.send(messaging.BearingAutopilotEnabledMessage(True, bearing))
        else:
            self.send(messaging.BearingAutopilotEnabledMessage(False, 0))

    def depth(self, depth):
        if depth is not None:
            self.send(messaging.DepthAutopilotEnabledMessage(True, depth))
        else:
            self.send(messaging.DepthAutopilotEnabledMessage(False, 0))

    def pitch(self, pitch):
        if pitch is not None:
            self.send(messaging.PitchAutopilotEnabledMessage(True, pitch))
        else:
            self.send(messaging.PitchAutopilotEnabledMessage(False, 0))

    def bearingParams(self, kp, ki, kd, scale):
        self.send(messaging.BearingAutopilotParamsMessage(kp, ki, kd, scale))

    def depthParams(self, kp, ki, kd, scale):
        self.send(messaging.DepthAutopilotParamsMessage(kp, ki, kd, scale))

    def pitchParams(self, kp, ki, kd, scale):
        self.send(messaging.PitchAutopilotParamsMessage(kp, ki, kd, scale))

    def prop(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.Prop, value))

    def hbow(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HBow, value))

    def vbow(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VBow, value))

    def hstern(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HStern, value))

    def vstern(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VStern, value))

    def v(self, value):
        self.vbow(value)
        self.vstern(value)

    def h(self, value):
        self.hbow(value)
        self.hstern(value)

    def r(self, value):
        self.hbow(value)
        self.hstern(-value)

    def checkRange(self, value):
        if value < -127 or value > 127:
            raise ValueError("invalid motor value: %d" % value)

    ## synchronous-ifying stuff
    def onStateMessage(self, m):
        self.received_state_condition.acquire()
        self.received_state = m
        self.received_state_condition.notify()
        self.received_state_condition.release()
