import cauv
import cauv.messaging as messaging
import cauv.control as control
import cauv.node
import threading
from math import degrees, cos, sin

class PipeFinder(messaging.BufferedMessageObserver):
    def __init__(self, node, auv, centre_name, target, error, strafe_p=30, depth_p=0.1):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        self.auv = auv
        node.join("processing")
        node.addObserver(self)
        self.centre_name = centre_name
        self.strafe_p = strafe_p
        self.depth_p = depth_p
        self.target = target
        self.error = error

    def onHoughLinesMessage(self, m):
        if len(m.lines):
            angle = sum([x.angle for x in m.lines])/len(m.lines)
            corrected_angle=degrees(angle)%180-90
            current_bearing = self.auv.getBearing()
            if current_bearing: #watch out for none bearings
                self.auv.bearing((current_bearing-corrected_angle)%360) #- as angle is opposite direction to bearing
            if len(m.lines) == 2:
                #
                width = abs(sin(angle)*(m.lines[0].centre.x-m.lines[1].centre.x)-cos(angle)*(m.lines[0].centre.y-m.lines[1].centre.y))
                if abs(width-self.target)>self.error:
                    dive=2*(width-self.target)
                    if self.auv.current_depth:
                        self.auv.depth(self.auv.current_depth+(self.depth_p)*abs(width-self.target))
                else:
                    dive=0
                print 'Turn: %f, Change in depth: %f' %(corrected_angle, dive)
            else:
                print 'Turn: %f, Not enough lines for depth calculations.' %(corrected_angle)

    def onCentreMessage(self, m):
        if m.name == self.centre_name:
            print 'Set strafe: %i' %(int((m.x-0.5)*self.proportion))
            self.auv.strafe(int((m.x-0.5)*self.proportion))

def setup():
    auv_node = cauv.node.Node('py-auv-pf')                #Create a node of the spread messaging service
    auv = control.AUV(auv_node)                        #Create a python object for the control of the AUV
    
    print 'setting calibration...'                #setting the y intercept and gradient of the pressure/depth curve for front and back pressure sensor
    # set-up calibration factors
    auv_node.send(messaging.DepthCalibrationMessage(
        -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    ), "control")

    auv.bearingParams(1, 0, -80, 1)                #Setting kp kd ki and scale of the following parameters
    auv.depthParams(40, 0, 0, 1)
    #auv.pitchParams(1, 0, 0, 1)

    auv.propMap(10, -10, 127, -127)
    auv.vbowMap(10, -10, 127, -127)
    auv.hbowMap(10, -10, 127, -127)
    auv.vsternMap(10, -10, 127, -127)
    auv.hsternMap(10, -10, 127, -127)

    node = cauv.node.Node('py-pf')
    pf = PipeFinder(node, auv, '', 0.4, 0.1)
    return pf
if __name__ == "__main__":
    setup
