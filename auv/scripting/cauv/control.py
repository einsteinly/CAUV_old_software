import messaging
import threading

class AUV(messaging.BufferedMessageObserver):
    def __init__(self, node):
        self.__node = node        
        node.join("control")
        node.addObserver(self)

    def send(self, msg):
        # send to control via self.__node
        self.__node.send(msg, "control")

    def stop(self):
        self.Prop(0)
        self.HBow(0)
        self.VBow(0)
        self.HStern(0)
        self.VStern(0) 
        self.autobearing(None)
        self.autoPitch(None)
        self.autoDepth(None)

    def autoBearing(self, bearing):
        if bearing is not None:
            self.send(messaging.BearingAutopilotEnabledMessage(True, bearing))
        else
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
        checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.Prop, value))

    def HBow(self, value):
        checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HBow, value))

    def VBow(self, value):
        checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VBow, value))

    def HStern(self, value):
        checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.HStern, value))

    def VStern(self, value):
        checkRange(value)
        self.send(messaging.MotorMessage(messaging.MotorID.VStern, value))

    def checkRange(self, value):
        if value < -127 || value > 127:
            raise ValueError("invalid motor value: %d" % value)

