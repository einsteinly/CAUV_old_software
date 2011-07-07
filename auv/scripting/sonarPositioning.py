#!/usr/bin/env python

import cauv
import cauv.messaging as msg
from math import cos,sin,atan2,pi,sqrt
import itertools
import cauv.node
import time


class vec:
    def __init__(self,x,y):
        self.x = x
        self.y = y
    def __add__(self, other):
        return vec(self.x+other.x, self.y+other.y)
    def __sub__(self, other):
        return vec(self.x-other.x, self.y-other.y)
    def __abs__(self):
        return sqrt(dotProd(self,self))
    def __repr__(self):
        return "(%f,%f)"%(self.x,self.y)
    

class lineSeg:
    def __init__(self,p1,p2):
        self.centre = p1
        self.angle = atan2(p2.y-p1.y, p2.x-p1.x)

def dotProd(p1,p2):
    return p1.x*p2.x + p1.y*p2.y

def crossProd(p1,p2):
    return p1.x*p2.y - p1.y*p2.x

def lineIntersection(line1, line2):
    if (line1.angle == line2.angle):
        return line1.centre
    
    p1 = vec(line1.centre.x, line1.centre.y)
    p2 = vec(line2.centre.x, line2.centre.y)
    v1 = vec(cos(line1.angle), sin(line1.angle))
    v2 = vec(cos(line2.angle), sin(line2.angle))
    v1xv2 = crossProd(v1, v2)
    p2p1xv2 = crossProd(vec(p2.x - p1.x, p2.y - p1.y), v2)

    s = p2p1xv2 / v1xv2

    return vec(p1.x + s * v1.x, p1.y + s * v1.y)

def pointLineDistance(p,l):
    p = vec(p.x,p.y)
    return abs(dotProd(p - l.centre, vec(-sin(l.angle), cos(l.angle))))

def nearestLinePoint(p,l):
    p = vec(p.x,p.y)
    v = vec(cos(l.angle), sin(l.angle))
    s = dotProd(p - l.centre, v)
    return vec(l.centre.x + s * v.x, l.centre.y + s * v.y)

def mergeLines(line1, line2):
    c = lineIntersection(line1, line2)
    
    a1 = normAngle(line1.angle)
    a2 = normAngle(line2.angle)
    
    l = msg.Line(msg.floatXYZ(c.x,c.y,0), (a1*line1.length+a2*line2.length)/(line1.length + line2.length), (line1.length+line2.length)/2)
    c2 = lineIntersection(l, lineSeg(line1.centre, line2.centre))
    l.centre.x = c2.x
    l.centre.y = c2.y
    return l

def normAngle(a):
    while a > pi/2:
        a = a - pi
    while a <= -pi/2:
        a = a + pi
    return a

def angleDiff(a1,a2):
    a1 = normAngle(a1)
    a2 = normAngle(a2)
    diff = (a1 - a2) % pi;
    return normAngle(diff);
    

def lineDistance(line1, line2):
    l1d = pointLineDistance(line2.centre, line1)
    l2d = pointLineDistance(line1.centre, line2) 
    return (l1d*line1.length+l2d*line2.length)/(line1.length + line2.length)


def positionInBay(lines, angleEpsilon=0.3, distanceEpsilon=0.1):
    if len(lines) < 3:
        print "Not enough lines"
        return None
    
    mergedLines = []
    for l1 in lines:
        merged = False
        for l2 in mergedLines:
            if abs(angleDiff(l1.angle, l2.angle)) < angleEpsilon and lineDistance(l1,l2) < distanceEpsilon:
                idx = mergedLines.index(l2)
                mergedLines[idx] = mergeLines(l1,l2)
                merged = True
                break
        if not merged:
            mergedLines.append(l1)
    if len(mergedLines) != 3:
        return None
    
    centre = vec(0.5,0.5)
    for p in itertools.permutations(mergedLines,3):
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
                return vec(abs(vSide1), abs(vBack))

    
    return None


if __name__ == "__main__":
    node = cauv.node.Node('py-sonPos')
    
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


