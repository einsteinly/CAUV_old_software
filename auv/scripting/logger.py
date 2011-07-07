#!/usr/bin/env python
# log-file writing process

import cauv
import cauv.node as node
import cauv.messaging as msg
from cauv.debug import debug, error, warning, info

import math
import time
import traceback

class LoggingObserver(msg.MessageObserver):
    def __init__(self, node):
        msg.MessageObserver.__init__(self)
        self.__node = node
        self.__node.addObserver(self)
        self.log = open(time.strftime('cauv-mission-%H-%M-%S.log'), 'w')
        self.__node.join("telemetry")
        self.__node.join("ai")
        # Properties:
        self.start_time = time.time()
        # From LocationMessage:
        self.latitude  = 0
        self.longitude = 0
        self.altitude  = 0
        self.speed = msg.floatXYZ()
        # From TelemetryMessage:
        self.depth = 0
        self.bearing = 0
        # From AILogMessage:
        self.comment = ""
    
    def close(self):
        self.log.close()

    def writeLine(self):
        now = time.time() - self.start_time
        line = "'(%g,%g,%g,%g,'%s')'\n" % (now, self.latitude, self.longitude, self.depth, self.comment)
        debug(line)
        self.log.write(line)
        self.comment = ''

    #def onScriptMessage(self, m):
    #    self.comment = 'executing new mission: %d bytes' % len(m.msg)
    #    self.writeLine()

    def onAIlogMessage(self, m):
        self.comment = str(m.msg)
        self.writeLine()
    
    def onTelemetryMessage(self, m):
        self.bearing = m.orientation.yaw
        self.depth = m.depth
        self.writeLine()

    def onLocationMessage(self, m):
        self.latitude = m.latitude
        self.longitude = m.longitude
        self.altitude = m.altitude
        self.speed = m.speed
        self.writeLine()

if __name__ == '__main__':
    n = node.Node("py-log")
    logger = LoggingObserver(n)
    try:
        while True:
            time.sleep(1.0)
    except Exception, e:
        error(str(traceback.format_exc()))
    finally:
        logger.close()

