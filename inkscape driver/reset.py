import device

e = device.WCB()
if not e.setupSerial():
    print "exiting"
    exit()

e.disable()
print "motors disabled"
