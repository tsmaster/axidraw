import device


e = device.WCB()
if not e.setupSerial():
        exit()
e.setupCommand(True)

e.EnableMotors()
e.penUp()
e.penDown()
e.penUp()

e.moveRel(4.0, 4.0, False)
e.penDown()
e.moveRel(1.0, 1.0, False)
