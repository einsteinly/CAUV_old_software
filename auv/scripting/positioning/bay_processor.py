#!/usr/bin/env python2.7

from cauv.debug import debug, warning, error, info

import cauv.sonar

from AI_location import aiLocationProvider, vec, dotProd
import cauv
import cauv.messaging as msg
from math import cos,sin,atan2,pi,sqrt
import itertools
import cauv.node
import time
import threading

import cauv.pipeline as pipeline

def crossProd(p1,p2):
    return p1.x*p2.y - p1.y*p2.x

def nearestLinePoint(p,l):
    p = vec(p.x,p.y)
    v = vec(cos(l.angle), sin(l.angle))
    s = dotProd(p - l.centre, v)
    return vec(l.centre.x + s * v.x, l.centre.y + s * v.y)

def angleDiff(a1,a2):
    diff = (a1 - a2) % pi;
    if diff > pi/2:
        diff = pi - diff
    return diff
    
def positionInBay(lines, angleEpsilon=0.3, distanceEpsilon=0.1):
    if len(lines) != 3:
        return None
    
    centre = vec(0.5,0.5)
    for p in itertools.permutations(lines,3):
        side1 = p[0]
        side2 = p[1]
        backWall = p[2]
       
        if angleDiff(p[0].angle, p[1].angle) < angleEpsilon and angleDiff(p[0].angle, p[2].angle + pi/2) < angleEpsilon:
            
            pBack = nearestLinePoint(centre, backWall)
            pSide1 = nearestLinePoint(centre, side1)
            pSide2 = nearestLinePoint(centre, side2)
            
            vBack = pBack - centre
            vSide1 = pSide1 - centre
            vSide2 = pSide2 - centre
            
            BxS1 = crossProd(vBack, vSide1)
            BxS2 = crossProd(vBack, vSide2)
            
            if BxS1 < 0 and BxS2 > 0:
                return vec(abs(vSide2), abs(vBack))

    return None


class locationProvider(aiLocationProvider):
    
    def __init__(self, node, args):
        aiLocationProvider.__init__(self, node)
        
        self.pipeline = pipeline.Model(node, "sonar")
        self.sonar = cauv.sonar.Sonar(node)
    
        self.sonarRange = 60000
        self.sonarRangeRes = 100
        self.sonarAnglularRes = 60
        self.sonarGain = 255
        self.sonarWidth = 6400
        
        self.lastPos = vec(0,0)
        self.alpha = 0.1
        
        self.linesMessageCount = 0
        
        self.positioningFinished = threading.Event()
        
    def fixPosition(self):
        # the AUV gets stopped in AI_location before this
        # method is called so we don't need to do it here
    
        # set up the sonar params
        self.sonar.range(self.sonarRange)
        self.sonar.rangeRes(self.sonarRangeRes)
        self.sonar.angularRes(self.sonarAnglularRes)
        self.sonar.gain(self.sonarGain)
        self.sonar.width(self.sonarWidth)
        
        # set up the pipeline
        self.pipeline.load("pipelines/sonar_walls.pipe")
        
        # wait for a full scan, but also check if we've run over
        # the time we're allowed for a capture
        while not self.positioningFinished.is_set():
            # self.stopped is set by AI_location if the 
            # auv pause we requested times out
            if self.stopped.is_set():
                error("Bay processing stopped by timeout")    
                return
            else: 
                debug("Still waiting for position fix")
                self.positioningFinished.wait(1)           
        
    def getPosition(self, pos):
        # convert to sonar range scale
        return vec(self.sonarRange*2 * pos.x / 1000.0, self.sonarRange*2 * pos.y / 1000.0)

    def onLinesMessage(self, m):
        if m.name == "bay lines":
        
            # we need some way to know when the positioning has been completed
            # as there's going to be noise in the reading we need to wait for
            # a few before returning a position
            self.linesMessageCount += 1
            if (self.linesMessageCount > 20): # wait for 20 messages
                self.linesMessageCount = 0
                self.positioningFinished.set()
            else: self.positioningFinished.clear()
                        
            # work out where we are from the lines...    
            rawpos = positionInBay(m.lines)                
            if rawpos != None:
                a = vec(self.lastPos.x * (1-self.alpha), self.lastPos.y * (1-self.alpha))
                b = vec(rawpos.x * (self.alpha), rawpos.y * (self.alpha))
                pos = a + b
                if self.lastPos == None:
                    self.lastPos = rawpos
                else: self.lastPos = pos
                info("Latest position: " + self.lastPos)
                
    def onSonarControlMessage(self, m):
        self.sonarRange = m.range


if __name__ == "__main__":
    import sys
    node = cauv.node.Node('py-sonPos',sys.argv[1:])
    
    class TestObserver(msg.MessageObserver):
        sonarRange = 60000
        lastPos = vec(0,0)
        alpha = 0.1

        def onLinesMessage(self, m):
            if m.name == "bay lines":
                rawpos = positionInBay(m.lines)                
                if rawpos != None:
                    a = vec(self.lastPos.x * (1-self.alpha), self.lastPos.y * (1-self.alpha))
                    b = vec(rawpos.x * (self.alpha), rawpos.y * (self.alpha))
                    pos = a + b
                    self.lastPos = pos
                    realpos = vec(self.sonarRange*2 * pos.x / 1000.0, self.sonarRange*2 * pos.y / 1000.0)
                    print rawpos, realpos
        def onSonarControlMessage(self, m):
            self.sonarRange = m.range
    
    node.addObserver(TestObserver())
    node.join("processing")
    node.join("sonarctl")
    while True:
        time.sleep(1.0)


