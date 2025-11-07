#!/usr/bin/env python3
import os
import rospy
import math
import serial
from puppy_control.msg import Velocity, Pose, Gait
from puppy_control.srv import SetRunActionName

BANNER = """
**********************************************************
Function: voice interaction routine (+ ActionGroups)
----------------------------------------------------------
Press Ctrl+C to exit.
----------------------------------------------------------
"""

# Initial posture & gait
PUPPY_POSE0 = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'height': -10, 'x_shift': -0.5, 'stance_x': 0, 'stance_y': 0}
GAIT0 = {'overlap_time': 0.2, 'swing_time': 0.2, 'clearance_time': 0.0, 'z_clearance': 3}

# --- Frame -> action group filename (include extension exactly as on disk) ---
FRAME_TO_ACTION = {
    "AA 55 00 80 FB": "1.d6a",
    "AA 55 00 81 FB": "2_legs_stand.d6ac",
    "AA 55 00 82 FB": "bow.d6ac",
    "AA 55 00 83 FB": "boxing.d6ac",
    "AA 55 00 84 FB": "boxing2.d6ac",
    "AA 55 00 85 FB": "demo.d6ac",
    "AA 55 00 86 FB": "grab.d6a",
    "AA 55 00 87 FB": "jump.d6ac",
    "AA 55 00 88 FB": "kick_ball_left.d6ac",
    "AA 55 00 89 FB": "kick_ball_right.d6ac",
    "AA 55 00 A1 FB": "lie_down.d6ac",
    "AA 55 00 A2 FB": "look_down.d6ac",     # prefer the .d6ac variant
    "AA 55 00 A3 FB": "moonwalk.d6ac",
    "AA 55 00 A4 FB": "nod.d6ac",
    "AA 55 00 A5 FB": "pee.d6ac",
    "AA 55 00 A6 FB": "press-up.d6ac",
    "AA 55 00 A7 FB": "push-up.d6ac",
    "AA 55 00 A8 FB": "shake_hands.d6ac",
    "AA 55 00 A9 FB": "shake_head.d6ac",
    "AA 55 00 AA FB": "sit.d6ac",
    "AA 55 00 AB FB": "spacewalk.d6ac",
    "AA 55 00 AC FB": "stand.d6ac",         # prefer the .d6ac variant
    "AA 55 00 AD FB": "stretch.d6ac",
    "AA 55 00 AE FB": "up_stairs_2cm.d6ac",
    "AA 55 00 AF FB": "up_stairs_3.5cm.d6ac",
    "AA 55 00 B0 FB": "wave.d6ac",
    "AA 55 00 B1 FB": "push-up01.d6ac",
    "AA 55 00 B2 FB": "stand_with_arm.d6a",
    "AA 55 00 B3 FB": "Clamping.d6a",
    "AA 55 00 B4 FB": "arm_test.d6a",
}

STOP_CODE      = "AA 55 00 09 FB"
ATTENTION_CODE = "AA 55 00 0A FB"
LIEDOWN_CODE  = "AA 55 00 0B FB"
LOOKUP_CODE   = "AA 55 00 8D FB"
MARCH_CODE    = "AA 55 00 76 FB"

run_st = True
def on_shutdown():
    global run_st
    run_st = False
    rospy.loginfo("Shutting down...")

def hex5(b):
    return ' '.join(f'{x:02X}' for x in b)

def read_fixed_frame(ser):
    """Read exactly one 5-byte token; return hex string or None on timeout/mismatch."""
    pkt = ser.read(5)
    if len(pkt) != 5: 
        return None
    if pkt[0] != 0xAA or pkt[1] != 0x55 or pkt[-1] != 0xFB:
        return None
    return hex5(pkt)

def run_action_group(callable_srv, filename_with_ext):
    """Call service with filename. Some images take (name) only; others (name, wait)."""
    try:
        # Most builds: single-string argument
        callable_srv(filename_with_ext)
    except TypeError:
        # Fallback if the service expects (name, wait)
        callable_srv(filename_with_ext, False)

def parse_and_dispatch(hex_data, pose_pub, vel_pub, run_ag_srv):
    # 1) Action groups
    if hex_data in FRAME_TO_ACTION:
        run_action_group(run_ag_srv, FRAME_TO_ACTION[hex_data])
        rospy.loginfo("ActionGroup -> %s", FRAME_TO_ACTION[hex_data])
        return

    # 2) Simple legacy motions kept for convenience
    if hex_data == MARCH_CODE:
        pose = dict(PUPPY_POSE0)
        pose_pub.publish(**pose, run_time=500)
        rospy.sleep(0.5)
        vel_pub.publish(x=0.1, y=0.0, yaw_rate=0.0)
        rospy.sleep(2)
        vel_pub.publish(x=0.0, y=0.0, yaw_rate=0.0)

    elif hex_data == ATTENTION_CODE:
        pose = dict(PUPPY_POSE0)
        pose_pub.publish(**pose, run_time=500)

    elif hex_data == LIEDOWN_CODE:
        pose = dict(PUPPY_POSE0); pose['height'] = -6
        pose_pub.publish(**pose, run_time=500)

    elif hex_data == LOOKUP_CODE:
        pose = dict(PUPPY_POSE0); pose['pitch'] = math.radians(20)
        pose_pub.publish(**pose, run_time=500)

    elif hex_data == STOP_CODE:
        vel_pub.publish(x=0.0, y=0.0, yaw_rate=0.0)
        global run_st; run_st = False

if __name__ == "__main__":
    print(BANNER)
    rospy.init_node('voice_interaction_demo_ag')
    rospy.on_shutdown(on_shutdown)

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    rospy.sleep(0.5)

    # Base posture & gait
    PuppyPosePub.publish(**PUPPY_POSE0, run_time=500)
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(**GAIT0)

    # ActionGroup service
    rospy.wait_for_service('/puppy_control/runActionGroup', timeout=5)
    RunAG = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)

    # Serial device
    dev = os.getenv("WONDERECHO_USB", "/dev/ttyUSB0")
    ser = serial.Serial(dev, 115200, timeout=0.2)
    rospy.loginfo("Using USB: %s", dev)

    while run_st and not rospy.is_shutdown():
        code = read_fixed_frame(ser)
        if code:
            rospy.loginfo("RX %s", code)
            parse_and_dispatch(code, PuppyPosePub, PuppyVelocityPub, RunAG)
        else:
            rospy.sleep(0.02)
