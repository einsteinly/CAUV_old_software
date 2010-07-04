#!/usr/bin/env python
# log-file writing process

import cauv
import cauv.node as node
import cauv.messaging as msg

import math
import time
import traceback

prop_speed_conversion = 0.4 / 127
strafe_speed_conversion = 0.6 / 127

class LoggingObserver(msg.BufferedMessageObserver):
    def __init__(self, node):
        msg.BufferedMessageObserver.__init__(self)
        self.__node = node
        self.__node.addObserver(self)
        self.log = open(time.strftime('cauv-%H-%M-%S.log'), 'w')
        self.__node.join("telemetry")
        self.__node.join("ai")
        self.__node.join("processing")
        self.__node.join("gui")

        self.yaw = None
        self.pitch = None
        self.roll = None
        self.start_time = time.time()
        self.last_time = 0
        self.speed_forwards = 0
        self.speed_right = 0
        self.x = 0
        self.y = 0
        self.depth = 0
        self.action = ""
    
    def close(self):
        self.log.close()

    def writeLine(self):
        t = time.time() - self.start_time
        dt = t - self.last_time
        self.x += dt * self.speed_forwards * math.cos(math.radians(self.yaw))
        self.y += dt * self.speed_forwards * math.sin(math.radians(self.yaw))
        self.x += dt * self.speed_right * math.sin(math.radians(self.yaw))
        self.y += -dt * self.speed_right * math.cos(math.radians(self.yaw))
        
        self.last_time = t
        line = "'(%g,%g,%g,%g,'%s')'" % (t, self.x, self.y, self.depth, self.action)
        print line
        self.log.write(line)

    def onScriptMessage(self, m):
        self.action = 'executing new mission: %d bytes' % len(m.msg)
        self.writeLine()

    def onBearingAutopilotParamsMessage(self, m):
        if m.enabled:
            self.action = 'new bearing set: %g' % m.target
        else:
            self.action = 'bearing control disabled'
        self.writeLine() 

    def onDepthAutopilotParamsMessage(self, m):
        if m.enabled:
            self.action = 'new depth set: %g' % m.target
        else:
            self.action = 'depth control disabled'
        self.writeLine()

    def onPitchAutopilotParamsMessage(self, m):
        if m.enabled:
            self.action = 'new pitch set: %g' % m.target
        else:
            self.action = 'pitch control disabled'
        self.writeLine()

    def onMotorStateMessage(self, m):
        self.speed_forwards = m.demand.prop * prop_speed_conversion
        net_h_power = m.demand.hbow + m.demand.hstern
        self.speed_right = net_h_power * strafe_speed_conversion
        self.writeLine()

    def onTelemetryMessage(self, m):
        self.yaw = m.orientation.yaw
        self.pitch = m.orientation.pitch
        self.roll = m.orientation.roll
        self.depth = m.depth
        self.writeLine()
    
    def onHoughLinesMessage(self, m):
        if len(m.lines):
            anglemsg = ''
            if m.lines[0].angle < 0:
                anglemsg = 'angled left at apparent %d degrees' % (-m.lines[0].angle)
            else:
                anglemsg = 'angled right at apparent %d degrees' % (m.lines[0].angle)
            if m.lines[0].centre > 0.5:
                self.action = 'pipeline detected right (%g,%g), %s' % (m.lines[0].centre.x, m.lines[0].centre.y, anglemsg)
            else:
                self.action = 'pipeline detected left (%g,%g), %s' % (m.lines[0].centre.x, m.lines[0].centre.y, anglemsg)
            self.writeLine()
    
    def onHoughCirclesMesage(self, m):
        if len(m.circles):
            if m.circles[0].centre > 0.5:
                self.action = 'buoy detected right (%g)' % (m.circles[0].centre)
            else:
                self.action = 'buoy detected left (%g)' % (m.circles[0].centre)

if __name__ == '__main__':
    n = node.Node("py-log")
    logger = LoggingObserver(n)
    
    try:
        while True:
            time.sleep(500)
    except Exception, e:
        traceback.print_exc()
        logger.close()

