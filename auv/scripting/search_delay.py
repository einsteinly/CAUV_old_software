#!/usr/bin/env python
# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node
from colourfinder2 import ColourFinder
from pipe_follow import PipeFinder
from pipe_confirm import PipeConfirmer

import time
import traceback
import math

def Search():

    node = cauv.node.Node('py-search')              #Create a node of the spread messaging service
    auv = control.AUV(node)                         #Create a python object for the control of the AUV
    yellowFinder = ColourFinder(node, [11, 12])     #Turn on the colour detection script
    confirmer = PipeConfirmer(node, auv, [11, 12]) #Holds the pipe confirm sequence
    
    
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
        time.sleep(10)
        print 'diving...'
        #auv.depthAndWait(depth)                                 #make sure it is at depth 2m
        auv.depth(depth)
        time.sleep(10)
        print 'spiral...'

        for i in range(1, 2*revolution):                        #making individual half revolutions
            print 'Performing %dth half circle' % i

            for j in range(2):                              #perform 2 turns for each half revolutions
                progress = 0                                #counter of progress in seconds
                startTime = time.time()                
                print 'Moving forward and searching for %d seconds' %(3*i)
                while progress < (unit*i):     #The time for which the AUV goes forward depends on the radius of the revolution
                    auv.prop(power)                
                    progress = time.time() - startTime    #Updating progress
                    
                    if yellowFinder.detected()==1:                   
                        #Quick stop
                        auv.prop(-127)
                        print 'found something, emergency stop'
                        time.sleep(2)
                        auv.prop(0)
                        
                        #Pipe confirmation
                        if confirmer.confirm()==False:
                        #if pipe is not found, continue the search, and note the new startTime
                            startTime = time.time()
                       
                        else:
                            #enable follower when pipe is confirmed
                            follower = PipeFinder(node, auv, 'pipe', 0.4, 0.1)    #Script to position the AUV to the center of yellowness and the adjust bearing
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
                time.sleep(10)
                
        print 'surface...'    
        #auv.depthAndWait(0)
        auv.depth(0)
        time.sleep(10)
        
    except Exception:
        traceback.print_exc()
        auv.depth(0)
        time.sleep(10)
        auv.stop()
    print 'Complete'

    return 0






if __name__ == '__main__':

    Search()
