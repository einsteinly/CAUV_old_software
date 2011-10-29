import cauv
import cauv.messaging as msg
import cauv.pipeline as pipeline
import cauv.control as control
import cauv.sonar
import cauv.node
import time
import sys

node = cauv.node.Node('sonartest',sys.argv[1:])
auv = control.AUV(node)
sonar = cauv.sonar.Sonar(node)
pl = pipeline.Model(node)


def bitVal(val, bit, off, on):
    if val & bit == bit:
        return on
    else:
        return off

def sendSonarTest(off, on, sleeptime = 0.05):
    print("Sending sonar test data")
    for b in range(0,64):
        line = msg.SonarDataLine()
        line.data.clear()
        for i in range(0,6):
            line.data.append(bitVal(b, 1 << i, off, on))
        line.bearing = b * 100
        line.bearingRange = 100
        node.send(msg.SonarDataMessage(line))
        time.sleep(sleeptime)


while True:
    sendSonarTest(80, 240)
    sendSonarTest(20, 140)

