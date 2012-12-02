import cauv
import cauv.node as node
import cauv.messaging as msg
from cauv.debug import debug, warning, error, info

import time
import random
import sys

n = node.Node("sonar-t",sys.argv[1:])

data_line = msg.SonarDataLine()
data_line.bearing = 0
data_line.bearingRange = 6400/32
data_line.range = 10

while True:
    data_line.data = msg.byteVec()
    for i in xrange(0, 50):
        data_line.data.append(i + random.randrange(0, 50) % 0xff)

    data_line.bearing += data_line.bearingRange
    if(data_line.bearing > 6400):
        data_line.bearing -= 6400
    info('%s' % data_line.bearing)

    n.send(msg.SonarDataMessage(data_line), "sonarout")
    time.sleep(0.05)


