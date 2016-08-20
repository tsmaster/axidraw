import device

e = device.WCB()
if not e.setupSerial():
    print "exiting"
    exit()

e.penDown()
print "pen down"
