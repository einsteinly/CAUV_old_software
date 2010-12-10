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
import math

def Search():

    node = cauv.node.Node('py-search')              #Create a node of the spread messaging service
    auv = control.AUV(node)                         #Create a python object for the control of the AUV
    yellowFinder = ColourFinder(node, [11, 12])     #Turn on the colour detection script
    yellowFinderCV = threading.Lock()               #Locking thing???
    follower = PipeFinder(node, auv, 'pipe', 0.4, 0.1)    #Script to position the AUV to the center of yellowness and the adjust bearing
    
    
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

    #Searc parameters
    revolution = 2                                  #Number of revolutions of the spiral square search from center
    bearing = 0                                     #Initial bearing of the search
    power = 64                                      #The motor power during search
    unit = 3                                        #The time length of the smallest half revolution, the time length of subsequence half revolution will be mulituple of this one
    depth = 0.5                                     #The depth of the search

    try:
        print 'setting bearing %d...' %bearing
        #auv.bearingAndWait(bearing)                         #Starting search at north direction
        auv.bearing(bearing)
        print 'diving...'
        #auv.depthAndWait(depth)                                 #make sure it is at depth 2m
        auv.depth(depth)
        print 'spiral...'

        for i in range(1, 2*revolution):                        #making individual half revolutions
            print 'Performing %dth half circle' % i

            for j in range(2):                              #perform 2 turns for each half revolutions
                auv.prop(power)
                print 'Moving forward and searching for %d seconds' %(3*i)
                startTime = time.time()
                while (time.time()-startTime) < unit*i:     #The time for which the AUV goes forward depends on the radius of the revolution
                    if yellowFinder.detect == None:                #There might be a threading problem here
                        yellowFinderCV.acquire()
                        flag = yellowFinder.detect
                        yellowFinderCV.release()
                    flag = yellowFinder.detect
                    
                    #print 'Detect flag is:',flag
                    if flag==1:                   
                        auv.prop(-127)
                        print 'found something, qick stop'
                        time.sleep(2)
                        auv.prop(0)
                        follower.enable=1
                        time.sleep(20)
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
