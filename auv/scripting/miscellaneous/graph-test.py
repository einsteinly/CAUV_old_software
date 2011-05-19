#!/usr/bin/env python
# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from cauv.debug import debug, warning, error, info

import time
import traceback
import math

import re

YPR_re = re.compile('.*\{ yaw = ([-0-9.]+), pitch = ([-0-9.]+), roll = ([-0-9.]+) \}.*')

if __name__ == '__main__':
    node = cauv.node.Node('py-fake')
    auv = control.AUV(node)
        

    with open('/Users/james/2010-12-10-control.log') as clog:
        for line in clog:
            time.sleep(0.01)
            if YPR_re.match(line):
                groups = YPR_re.match(line).groups()
                print 'ypr:', groups
                node.send(msg.TelemetryMessage(
                    msg.floatYPR(
                        float(groups[0]),
                        float(groups[1]),
                        float(groups[2])),
                    0.0))
