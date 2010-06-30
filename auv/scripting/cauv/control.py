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
        self.Prop(0)
        self.HBow(0)
        self.VBow(0)
        self.HStern(0)
        self.VStern(0)
        self.autoBearing(None)
        self.autoPitch(None)
        self.autoDepth(None)

    def bearing(self, timeout=3):
        self.received_state_condition.acquire()
        self.received_state = None
        self.send(messaging.StateRequestMessage())
        self.received_state_condition.wait(timeout)
        self.received_state_condition.release()
        return self.received_state.orientation

    def autoBearing(self, bearing):
        if bearing is not None:
            self.send(messaging.BearingAutopilotEnabledMessage(True, bearing))
        else:
            self.send(messaging.BearingAutopilotEnabledMessage(False, 0))

    def autoDepth(self, depth):
        if depth is not None:
            self.send(messaging.DepthAutopilotEnabledMessage(True, depth))
        else:
            self.send(messaging.DepthAutopilotEnabledMessage(False, 0))

    def autoPitch(self, pitch):
        if pitch is not None:
            self.send(messaging.PitchAutopilotEnabledMessage(True, pitch))
        else:
            self.send(messaging.PitchAutopilotEnabledMessage(False, 0))

    def autoBearingParameters(self, kp, ki, kd, scale):
        self.send(messaging.BearingAutopilotParamsMessage(kp, ki, kd, scale))

    def autoDepthParameters(self, kp, ki, kd, scale):
        self.send(messaging.DepthAutopilotParamsMessage(kp, ki, kd, scale))

    def autoPitchParameters(self, kp, ki, kd, scale):
        self.send(messaging.PitchAutopilotParamsMessage(kp, ki, kd, scale))

    def Prop(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.Prop, value))

    def HBow(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HBow, value))

    def VBow(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VBow, value))

    def HStern(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HStern, value))

    def VStern(self, value):
        self.checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VStern, value))

    def V(self, value):
        self.VBow(value)
        self.VStern(value)

    def H(self, value):
        self.HBow(value)
        self.HStern(value)

    def checkRange(self, value):
        if value < -127 or value > 127:
            raise ValueError("invalid motor value: %d" % value)

    ## synchronous-ifying stuff
    def onStateMessage(self, m):
        self.received_state_condition.acquire()
        self.received_state = m
        self.received_state_condition.notify()
        self.received_state_condition.release()
