#!/usr/bin/env python
# log-file writing process

import cauv.messaging as msg
import cauv.node as node

from cauv.debug import debug, info, warning, error

import time

from gps import *

class GPSNode():
    def __init__(self, node):
        self.__node = node
        self.session = gps() 
        self.session.stream(WATCH_ENABLE|WATCH_NEWSTYLE)

    def run(self):
        for report in self.session:
            info("Report received -- " + report["class"])
            if(report["class"] == "TPV"):
                info("Location: [lat %f, lon %f, alt %f]" % (report.lat, report.lon, report.alt))
                self.__node.send(msg.GPSLocationMessage(report.lat, report.lon, report.alt, report.track, report.speed, report.climb))

    
if __name__ == '__main__':
    try:
        n = node.Node("py-gps")
        while True:
            try:
                GPSNode(n).run()
            except StopIteration:
                info("GPS stopped, trying to restart")
    except socket.error:
        error("GPSD not running") 

