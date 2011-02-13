#!/usr/bin/env python
# down, forwards, and up

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.node

import time
import traceback


def Spiral(): 
    node = cauv.node.Node('py-spiral')                #Create a node of the spread messaging service
    auv = control.AUV(node)                        #Create a python object for the control of the AUV
    
    print 'setting calibration...'                #setting the y intercept and gradient of the pressure/depth curve for front and back pressure sensor
    # set-up calibration factors
    node.send(msg.DepthCalibrationMessage(
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

    time.sleep(2)
    #bearing = auv.getBearing()
    #if bearing is None:
    #    print 'no bearing information!'
    #    time.sleep(5)
    #    bearing = 90

    square=2
    bearing=0
    power=64
    unit = 3
    
    try:
        print 'setting bearing %d...' %bearing
        auv.bearing(bearing)            #Starting search at north direction
        time.sleep(10)

        print 'diving...'
        auv.depth(0.5)            #make sure it is at depth 0.5m
        time.sleep(10)
        
        print 'spiral...'
              
        for i in range(1, 2*square): #making individual half squares
            print 'Performing %dth half circle' % i

            for j in range(2): #making individual half squares
                progress = 0                                #counter of progress in seconds
                startTime = time.time()                
                print 'Moving forward and searching for %d seconds' %(3*i)
                while progress < (unit*i):     #The time for which the AUV goes forward depends on the radius of the revolution
                    auv.prop(power)                
                    progress = time.time() - startTime    #Updating progress
                    print 'Moving forward for %d seconds' %(3*i)
                auv.prop(0)            #shut off motor
                time.sleep(5)        #wait for the AUV to stop
                print 'stoping'           
                bearing += 90        #Turn 90 degree
                if bearing>=360:
                    bearing-=360
                print 'setting bearing %d' %bearing
                auv.bearing(bearing)
                time.sleep(10)
            
        print 'surface...'    
        auv.depth(0)
        time.sleep(10)

        
    except Exception:
        traceback.print_exc()
        auv.depth(0)
        auv.stop()
    print 'Complete'

if __name__ == '__main__':
    Spiral()




