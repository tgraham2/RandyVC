#!/usr/bin/env python3
import os
import rospy
import math
import serial
from std_msgs.msg import String
from puppy_control.msg import Velocity, Pose, Gait

BANNER = """
**********************************************************
Function: voice interaction routine
    Randypi Voice Control (RandyVC)
**********************************************************
"""

# Initial posture & gait
PUPPY_POSE0 = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'height': -10, 'x_shift': -0.5, 'stance_x': 0, 'stance_y': 0}
GAIT0 = {'overlap_time': 0.2, 'swing_time': 0.2, 'clearance_time': 0.0, 'z_clearance': 3}

run_st = True
def on_shutdown():
    global run_st
    run_st = False
    rospy.loginfo("Shutting down...")

def parse_serial_data(data):
    """Very simple matcher; consider a real frame parser with AA 55 ... FB."""
    hex_data = ' '.join(f"{b:02X}" for b in data)
    rospy.loginfo(f"Received data: {hex_data}")

    if hex_data == "AA 55 00 76 FB":  # March in place
        pose = dict(PUPPY_POSE0)
        PuppyPosePub.publish(**pose, run_time=500)
        rospy.sleep(0.5)
        PuppyVelocityPub.publish(x=0.1, y=0.0, yaw_rate=0.0)
        rospy.sleep(2)
        PuppyVelocityPub.publish(x=0.0, y=0.0, yaw_rate=0.0)

    elif hex_data == "AA 55 00 0A FB":  # Attention (stand)
        pose = dict(PUPPY_POSE0)
        PuppyPosePub.publish(**pose, run_time=500)

    elif hex_data == "AA 55 00 0B FB":  # Lie down
        pose = dict(PUPPY_POSE0); pose['height'] = -6
        PuppyPosePub.publish(**pose, run_time=500)

    elif hex_data == "AA 55 00 8D FB":  # Look up
        pose = dict(PUPPY_POSE0); pose['pitch'] = math.radians(20)
        PuppyPosePub.publish(**pose, run_time=500)

    elif hex_data == "AA 55 00 09 FB":  # Stop
        PuppyVelocityPub.publish(x=0.0, y=0.0, yaw_rate=0.0)
        global run_st; run_st = False

if __name__ == "__main__":
    print(BANNER)
    rospy.init_node('voice_interaction_demo')
    rospy.on_shutdown(on_shutdown)

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    rospy.sleep(0.5)

    PuppyPosePub.publish(**PUPPY_POSE0, run_time=500)
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(**GAIT0)

    dev = os.getenv("WONDERECHO_USB")
    if not dev:
        raise RuntimeError("WONDERECHO_USB not set (e.g., /dev/ttyACM0).")
    ser = serial.Serial(dev, 115200, timeout=0.1)
    rospy.loginfo("Using USB: %s", dev)

    while run_st and not rospy.is_shutdown():
        if ser.in_waiting > 0:
            parse_serial_data(ser.read_all())
        rospy.sleep(0.05)
