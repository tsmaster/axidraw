import mazegen
import random
import math


CELL_SIZE = 0.25
X_MIN = 0
Y_MIN = 0
X_MAX = 8.5
Y_MAX = 7.5

numXPoints = int((X_MAX - X_MIN) / CELL_SIZE) + 1
numYPoints = int((Y_MAX - Y_MIN) / CELL_SIZE) + 1

points = []
indexPoints = []
edges = []
cells = []

XASPECT = math.sqrt(3)/2.0

def indexPairToCartPair(x,y):
    cartX = x * XASPECT * CELL_SIZE
    cartY = (y + x * 0.5) * CELL_SIZE
    return (cartX, cartY)

def makePoints():
    for xi in range(2 * numXPoints):
        for yi in range(-numYPoints, numYPoints + 1):
            cx,cy = indexPairToCartPair(xi, yi)
            if ((cx >= X_MIN) and
                (cx <= X_MAX) and
                (cy >= Y_MIN) and
                (cy <= Y_MAX)):
                points.append((cx,cy))
                indexPoints.append((xi,yi))

def makeEdge(pi1, pi2):
    edgePair = (pi1, pi2)
    if pi2 < pi1:
        edgePair = (pi2, pi1)
    
    try:
        existingIndex = edges.index(edgePair)
        return existingIndex
    except ValueError:
        edges.append(edgePair)
        return len(edges)-1

def makeEdgesAndCells():
    for xi in range(-numXPoints, numXPoints + 1):
        for yi in range(-numYPoints, numYPoints + 1):
            if (yi - xi) % 3 != 0:
                continue
            
            index1 = (xi + 1, yi)
            index2 = (xi, yi + 1)
            index3 = (xi - 1, yi + 1)
            index4 = (xi - 1, yi)
            index5 = (xi, yi - 1)
            index6 = (xi + 1, yi - 1)

            pointIndices = [index1,
                            index2,
                            index3,
                            index4,
                            index5,
                            index6]

            if all([(p in indexPoints) for p in pointIndices]):
                #print "cell xi, yi exists", xi, yi

                i1 = indexPoints.index(index1)
                i2 = indexPoints.index(index2)
                i3 = indexPoints.index(index3)
                i4 = indexPoints.index(index4)
                i5 = indexPoints.index(index5)
                i6 = indexPoints.index(index6)

                e1 = makeEdge(i1, i2)
                e2 = makeEdge(i2, i3)
                e3 = makeEdge(i3, i4)
                e4 = makeEdge(i4, i5)
                e5 = makeEdge(i5, i6)
                e6 = makeEdge(i6, i1)
            
                cells.append([e1, e2, e3, e4, e5, e6])


def makeMaze():
    unvisited = cells[1:]
    visited = cells[:1]

    while unvisited:
        randCell = random.choice(visited)
        otherCell = tryRemoveWall(randCell, unvisited)
        if not (otherCell is None):
            visited.append(otherCell)
            unvisited.remove(otherCell)
            
def tryRemoveWall(cell, unvisited):
    #print "trying to remove a wall from cell:", cell
    walls = cell[:]
    random.shuffle(walls)

    for w in walls:
        c = findCellInUnvisitedWithWall(unvisited, w)
        if not (c is None):
            #print "I will remove", w
            cell.remove(w)
            c.remove(w)
            return c

    return None
    
def findCellInUnvisitedWithWall(unvisited, wall):
    for c in unvisited:
        if wall in c:
            return c
    return None

def trimEdges():
    global edges
    oldEdges = edges[:]
    edges = []
    for c in cells:
        for e in c:
            p1, p2 = oldEdges[e]
            makeEdge(p1,p2)
        
makePoints()
print "cartesian points", points
print "indexed points", indexPoints
makeEdgesAndCells()
print "initial edges:", edges
print "initial cells:", cells

makeMaze()
print "trimmed cells:", cells
trimEdges()
print "trimmed edges:", edges


#mazegen.drawEdges(points, edges)

strips = mazegen.makeEdgeStrips(edges)

print strips


#mazegen.drawEdges(points, edges)
mazegen.drawStrips(points, strips)

