import mazegen
import random

CELL_SIZE = 0.3
X_MIN = 0.5
Y_MIN = 0.5
X_MAX = 7.5
Y_MAX = 6.5

numXPoints = int((X_MAX - X_MIN) / CELL_SIZE) + 1
numYPoints = int((Y_MAX - Y_MIN) / CELL_SIZE) + 1

points = []
indexPoints = []
edges = []
cells = []

def makePoints():
    for xi in range(numXPoints):
        x = X_MIN + xi * CELL_SIZE
        for yi in range(numYPoints):
            y = Y_MIN + yi * CELL_SIZE
            points.append((x,y))
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
    for xi in range(numXPoints - 1):
        for yi in range(numYPoints - 1):
            pi1 = xi * numYPoints + yi
            pi2 = pi1 + 1
            pi4 = pi1 + numYPoints
            pi3 = pi4 + 1
            e1 = makeEdge(pi1, pi2)
            e2 = makeEdge(pi2, pi3)
            e3 = makeEdge(pi3, pi4)
            e4 = makeEdge(pi4, pi1)

            cells.append([e1, e2, e3, e4])

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
makeEdgesAndCells()
print "initial edges", edges
print "initial cells", cells

makeMaze()
print "trimmed cells:", cells
trimEdges()
print "trimmed edges:", edges

strips = mazegen.makeEdgeStrips(edges)

print "strips:", strips


#mazegen.drawEdges(points, edges)
mazegen.drawStrips(points, strips)

