ser = serial.Serial(dev, 115200, timeout=0.1)
while not rospy.is_shutdown():
    pkt = ser.read(5)
    if len(pkt) == 5:
        print("RX:", hex5(pkt))
