#!/usr/bin/env python
# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from colourfinder2 import ColourFinder
from pipe_follow import PipeFinder

import time
import traceback
import threading

def Search():
    node = cauv.node.Node('py-search')              #Create a node of the spread messaging service
    auv = control.AUV(node)                         #Create a python object for the control of the AUV
    detect = ColourFinder(node, [11, 12])                 #Turn on the colour detection script
    detectCV = threading.Condition()
            
    print 'setting calibration...'                  #setting the y intercept and gradient of the pressure/depth curve for front and back pressure sensor
    # set-up calibration factors
    node.send(msg.DepthCalibrationMessage(
        -912.2/96.2, 1.0/96.2, -912.2/96.2, 1.0/96.2
    ), "control")

    auv.bearingParams(1, 0, -80, 1)                 #Setting kp kd ki and scale of the following parameters
    auv.depthParams(40, 0, 0, 1)
    #auv.pitchParams(1, 0, 0, 1)

    auv.propMap(10, -10, 127, -127)
    auv.vbowMap(10, -10, 127, -127)
    auv.hbowMap(10, -10, 127, -127)
    auv.vsternMap(10, -10, 127, -127)
    auv.hsternMap(10, -10, 127, -127)

    time.sleep(2)
    #bearing = auv.getBearing()
    #if bearing is None:
    #    print 'no bearing information!'
    #    time.sleep(5)
    #    bearing = 90

    square = 2
    bearing = 0
    power = 64
    unit = 3
    depth = 0.5 

    try:
        print 'setting bearing %d...' %bearing
        #auv.bearingAndWait(bearing)                         #Starting search at north direction
        auv.bearing(bearing)
        print 'diving...'
        #auv.depthAndWait(depth)                                 #make sure it is at depth 2m
        auv.depth(depth)
        print 'spiral...'

        for i in range(1, 2*square):                        #making individual half squares
            print 'Performing %dth half circle' % i

            for j in range(2):                              #perform 2 turns for each half squares
                auv.prop(power)
                print 'Moving forward and searching for %d seconds' %(3*i)
                startTime = time.time()
                while (time.time()-startTime) < unit*i:     #The time for which the AUV goes forward depends on the radius of the square
                    #if detect.detect == None:                #There might be a threading problem here
                    #    detectCV.acquire()
                    #    detectCV.wait(0.01)
                    #    detectCV.release()
                    flag = detect.detect
                    #print 'Detect flag is:',flag
                    if flag==1:                   
                        auv.prop(-127)
                        print 'found something, qick stop'
                        time.sleep(2)
                        auv.prop(0)
                        pf = PipeFinder(node, auv, 'pipe', 0.4, 0.1)
                        print 'surface...'    
                        #auv.depthAndWait(0)   
                        auv.depth(0)   
                        return 0                            #Insert object confirmation and reaction sequence here later


                time.sleep(unit*i)        
                auv.prop(0)                                 #shut off motor
                time.sleep(5)                               #wait for the AUV to stop
                print 'stoping'           
                bearing += 90                               #Turn 90 degree
                if bearing>=360:    
                    bearing-=360
                print 'setting bearing %d' %bearing
                #auv.bearingAndWait(bearing)
                auv.bearing(bearing)
                
        print 'surface...'    
        #auv.depthAndWait(0)
        auv.depth(0)
        
    except Exception:
        traceback.print_exc()
        auv.depth(0)
        auv.stop()
    print 'Complete'

    return 0

if __name__ == '__main__':
    Search()
