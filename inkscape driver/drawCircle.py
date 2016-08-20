import device
import math

CENTER_X = 4.0
CENTER_Y = 3.0

RADIUS_STEP = 0.25
MAX_RADIUS = 3.5
ANGLE_STEPS = 8
STEP_DIST = 0.2

e = device.WCB()
if not e.setupSerial():
        exit()
e.setupCommand(True)

e.EnableMotors()

r = RADIUS_STEP
while r <= MAX_RADIUS:
    circumference = 2 * math.pi * r
    ANGLE_STEPS = int(math.floor(circumference / STEP_DIST))
    e.penUp()
    e.moveAbs(CENTER_X + r, CENTER_Y, False)
    e.penDown()
    for angleIndex in range(ANGLE_STEPS):
        angle = 2.0 * math.pi * angleIndex / float(ANGLE_STEPS)
        x = CENTER_X + math.cos(angle) * r
        y = CENTER_Y + math.sin(angle) * r
        print "abs target:", x, y
        e.moveAbs(x, y, False)
    e.penUp()
    r += RADIUS_STEP

e.moveAbs(0, 0, True)
e.disable()
print "done"
