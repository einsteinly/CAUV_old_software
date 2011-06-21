import cauv
import cauv.messaging as messaging
import cauv.control as control
import cauv.node
import threading
from math import degrees, cos, sin
import time
from cauv.debug import debug, info

class PipePositioner(messaging.BufferedMessageObserver):
    def __init__(self, node, auv, centre_name='pipe', target_width=0.2, width_error=0.1, centre_error=0.1, align_error=5, bin=11, threshold=0.1, strafe_p=255, depth_p=0.1, depth_enable=False):
        #basic setup
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node
        self.auv = auv
        node.join("processing")
        node.addObserver(self)
        #name of centre node in pipeline
        self.centre_name = centre_name
        #parameters to say if the auv is above the pipe
        self.alignment_lock = threading.Lock()
        self.ready_c = threading.Condition(self.alignment_lock) #condition to notify when ready state changes
        self.centred = False
        self.aligned = False
        self.depthed = False
        self.ready = False
        #proportions for strafing and diving (relative to pipe position and width respectively)
        self.strafe_p = strafe_p
        self.depth_p = depth_p
        #target width of pipe as a proportion the image
        self.target = target_width
        #acceptables limits to say we are over the pipe
        self.align_error = align_error
        self.depth_error = width_error
        self.centre_error_2 = centre_error**2 #saves squaring it later
        #enable allows centering and alignment of the sub
        self.enable_lock = threading.Condition(threading.Lock())
        self.enable = True
        #bin and min value of the bin from histogram node (used to decide whether to set enable=True)
        self.bin = bin
        self.threshold = threshold
        self.depth_enable = depth_enable

    def onLinesMessage(self, m):
        #print "lines message"
        #get current bearing asap, before we turn to much
        current_bearing = self.auv.getBearing()
        #check whether this is enabled
        #self.enable_lock.acquire()
        if len(m.lines):
            #self.enable_lock.release()
            #calculate the average angle of the lines (assumes good line finding)
            angle = sum([x.angle for x in m.lines])/len(m.lines)
            #calculate the bearing of the pipe (relative to the sub), but mod 180 as we dont want to accidentally turn the sub around
            corrected_angle=90-degrees(angle)%180
            #print "angle %f corrected angle %f" %(angle, corrected_angle)
            if current_bearing: #watch out for none bearings
                self.auv.bearing((current_bearing-corrected_angle)%360) #- as angle is opposite direction to bearing
            #we can only calculate width if we have 2 lines, and dont bother if the angle is too different
            if len(m.lines) >= 2 and degrees(abs(m.lines[0].angle-m.lines[1].angle)) < 15 and self.depth_enable:
                xpos = []
                ypos = []
                for r in m.lines:
                    xpos.append(r.centre.x)
                    ypos.append(r.centre.y)
                width = abs(sin(angle)*(max(xpos)-min(xpos))-cos(angle)*(max(ypos)-min(ypos))) #modulus of the cross product of the delta posistion of centres of 2 lines, and a unit vector of a line
                #print 'estimated width of pipe in image: %f' %(width)
                if abs(width-self.target)>self.depth_error:
                    dive=(self.depth_p)*(width-self.target)
                    if self.auv.current_depth: #again, none depths
                        self.auv.depth(self.auv.current_depth+dive)
                else:
                    dive=0
                #print logic, can be removed if neccessary
                info('Angle: %f, Pipe/Image ratio: %f, Turn: %f, Change in depth: %f' %(angle, width, corrected_angle, dive))
            else:
                dive=0
                info('Angle: %f, Turn: %f, Not enough lines or lines to different for depth estimate or depth not enabled.' %(angle, corrected_angle))
            #acquire lock over alignment statuses
            self.ready_c.acquire()
            #set aligned +depthed status
            self.aligned = abs(corrected_angle) < self.align_error
            self.depthed = bool(dive)
            debug('aligned %i, depthed %i' %(self.aligned,self.depthed))
            #if aligned and centered, then ready should be true, else it should be false
            #if this is different to what it currently is
            if (self.centred and self.aligned and self.depthed) ^ self.ready:
                self.ready = self.centred and self.aligned and self.depthed
                #notify anything thats listening that the state has changed
                self.ready_c.notify()
            self.ready_c.release()
        #else: #dont forget to release the lock...
        #    self.enable_lock.release()

    def onCentreMessage(self, m):
        #print 'centre message'
        #check if enabled
        #self.enable_lock.acquire()
        #if self.enable:
        #self.enable_lock.release()
        st = int((m.x-0.5)*self.strafe_p)
        self.auv.strafe(st)
        self.alignment_lock.acquire()
        self.centred = m.x**2 + m.y**2<self.centre_error_2 #ie within circle radius centre error
        debug(str(self.centred))
        info('Strafing %i' %(st))
        self.alignment_lock.release()
        #else: #dont forget to release lock, and make sure sub isnt still strafing when disabled
        #    self.auv.strafe(0)
        #    self.enable_lock.release()
    """
    def onHistogramMessage(self, m):
        #enable/disable based on histogram messages
        self.enable_lock.acquire()
        if (m.bins[self.bin] > self.threshold) ^ self.enable:
            self.enable = m.bins[self.bin] > self.threshold
            self.enable_lock.notify()
        self.enable_lock.release()
    """
    def cleanup(self):
        self.__node.removeObserver(self)

def pipe_follow(auv_node, auv, ready_timeout, lost_timeout = 2, following_timeout = None, prop_speed = 20, **kwargs):
    pf = PipePositioner(auv_node, auv, **kwargs)
    while True:
        pf.ready_c.acquire()
        if not pf.ready: pf.ready_c.wait(ready_timeout) #check if we are aligned, if not then wait until we are or timeout
        if not pf.ready:
            print "Took to long to align over pipe"
            pf.ready_c.release()
            pf.cleanup()
            return
        pf.ready_c.release()
        print "Above pipe, heading forward"
        auv.prop(prop_speed)
        pf.ready_c.acquire()
        if pf.ready: pf.ready_c.wait(following_timeout) #check we are still aligned, if so wait until we are no longer aligned
        pf.ready_c.release()
        print "No longer above pipe, attempting to re-align"
        auv.prop(0)
        #we are no longer aligned with the pipe, stop, and check whether we are still attempting to align (ie enable is still true)
        pf.enable_lock.acquire()
        if not pf.enable: pf.enable_lock.wait(lost_timeout) #if it isnt, give it lost_timeout to reenable
        if not pf.enable:
            print "Lost the pipe, giving up"
            pf.enable_lock.release()
            pf.cleanup()
            return
        pf.enable_lock.release()
        #we found the pipe again, loop around

def setup():
    auv_node = cauv.node.Node('py-auv-pf')                #Create a node of the spread messaging service
    auv = control.AUV(auv_node)                        #Create a python object for the control of the AUV
    
    print 'setting calibration...'                #setting the y intercept and gradient of the pressure/depth curve for front and back pressure sensor
    # set-up calibration factors
    #auv_node.send(messaging.DepthCalibrationMessage(
    #    -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    #), "control")

    #auv.bearingParams(1, 0, -80, 1)                #Setting kp kd ki and scale of the following parameters
    #auv.depthParams(40, 0, 0, 1)
    #auv.pitchParams(1, 0, 0, 1)

    #auv.propMap(10, -10, 127, -127)
    #auv.vbowMap(10, -10, 127, -127)
    #auv.hbowMap(10, -10, 127, -127)
    #auv.vsternMap(10, -10, 127, -127)
    #auv.hsternMap(10, -10, 127, -127)

    pipe_follow(auv_node, auv, 200)
    
if __name__ == "__main__":
    setup()
