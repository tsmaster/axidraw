import device
import math
import time
import random


"""
pointList is a list of (x,y) tuples.
edgeList is a list of (i1, i2) indices into the pointList.
"""
def drawEdges(pointList, edgeList):
    timeStart = time.time()
    e = device.WCB()
    if not e.setupSerial():
        exit()
    e.setupCommand(True)

    e.EnableMotors()

    e.penUp()
    time.sleep(0.5)
    e.penDown()
    time.sleep(0.5)
    e.penUp()

    for i1, i2 in edgeList:
        x1, y1 = pointList[i1]
        e.moveAbs(x1, y1, False)
        e.penDown()
        x2, y2 = pointList[i2]
        e.moveAbs(x2, y2, False)
        e.penUp()

    e.moveAbs(0, 0, True)
    e.disable()
    timeEnd = time.time()
    print "elapsed time: ", int(timeEnd-timeStart)


"""
Takes a list of (i1, i2) pairs that index into a point list (somewhere else).
Returns a list of lists (i1, i2, i3, ... in) that are strips.
"""
def makeEdgeStrips(edgeList):
    unusedEdges = edgeList[:]
    outputStrips = []
    random.shuffle(unusedEdges)
    
    while unusedEdges:
        i1,i2 = unusedEdges.pop(0)
        s1 = findStripStartingAt(i1, unusedEdges)
        unusedEdges = removeStripFromEdgeList(s1, unusedEdges)
        s2 = findStripStartingAt(i2, unusedEdges)
        unusedEdges = removeStripFromEdgeList(s2, unusedEdges)
        s1.reverse()
        outputStrips.append(s1 + s2)

    return outputStrips


def findStripStartingAt(i, unusedEdges):
    e = findEdgeContainingIndex(i, unusedEdges)
    if e is None:
        return [i]
    i1, i2 = e
    if i1 == i:
        otherIndex = i2
    else:
        otherIndex = i1

    myUnused = unusedEdges[:]
    myUnused.remove(e)

    return [i] + findStripStartingAt(otherIndex, myUnused)


def findEdgeContainingIndex(edgeIndex, edgeList):
    for e in edgeList:
        if edgeIndex in e:
            return e
    return None
    
def removeStripFromEdgeList(strip, unusedEdges):
    newEdges = unusedEdges[:]
    for i in range(len(strip)-1):
        v1 = strip[i]
        v2 = strip[i+1]

        if (v1,v2) in newEdges:
            newEdges.remove((v1, v2))
        else:
            newEdges.remove((v2, v1))
    return newEdges


def drawStrips(pointList, edgeStrips):
    timeStart = time.time()
    e = device.WCB()
    if not e.setupSerial():
        exit()
    e.setupCommand(True)

    e.EnableMotors()

    e.penUp()
    time.sleep(0.5)
    e.penDown()
    time.sleep(0.5)
    e.penUp()

    for s in edgeStrips:
        path = [list(pointList[i]) for i in s]
        p0x, p0y = pointList[s[0]]
        e.moveAbs(p0x, p0y, False)
        e.penDown()
        e.PlanTrajectory(path)
        e.penUp()

    e.moveAbs(0, 0, True)
    e.disable()
    timeEnd = time.time()
    print "elapsed time: ", int(timeEnd-timeStart)
    
