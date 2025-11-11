#!/usr/bin/env python3
import os
import rospy
import math
import serial
import threading
from puppy_control.msg import Velocity, Pose, Gait
from puppy_control.srv import SetRunActionName

BANNER = """
**********************************************************
Function: voice interaction routine (+ ActionGroups + NAV)
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
    "AA 55 00 A2 FB": "look_down.d6ac",
    "AA 55 00 A3 FB": "moonwalk.d6ac",
    "AA 55 00 A4 FB": "nod.d6ac",
    "AA 55 00 A5 FB": "pee.d6ac",
    "AA 55 00 A6 FB": "press-up.d6ac",
    "AA 55 00 A7 FB": "push-up.d6ac",
    "AA 55 00 A8 FB": "shake_hands.d6ac",
    "AA 55 00 A9 FB": "shake_head.d6ac",
    "AA 55 00 AA FB": "sit.d6ac",
    "AA 55 00 AB FB": "spacewalk.d6ac",
    "AA 55 00 AC FB": "stand.d6ac",
    "AA 55 00 AD FB": "stretch.d6ac",
    "AA 55 00 AE FB": "up_stairs_2cm.d6ac",
    "AA 55 00 AF FB": "up_stairs_3.5cm.d6ac",
    "AA 55 00 B0 FB": "wave.d6ac",
    "AA 55 00 B1 FB": "push-up01.d6ac",
    "AA 55 00 B2 FB": "stand_with_arm.d6a",
    "AA 55 00 B3 FB": "Clamping.d6a",
    "AA 55 00 B4 FB": "arm_test.d6a",
}

# --- Extra fixed codes ---
STOP_CODE       = "AA 55 00 09 FB"
ATTENTION_CODE  = "AA 55 00 0A FB"   # repurposed as QUIT
LIEDOWN_CODE    = "AA 55 00 0B FB"
LOOKUP_CODE     = "AA 55 00 8D FB"
MARCH_CODE      = "AA 55 00 76 FB"

# --- NEW: Motion map (F1–F4 timed motions) ---
MOTION_MAP = {
    "AA 55 00 F1 FB": {"x":  0.12, "y": 0.0,  "yaw_rate": 0.0,  "duration": 2.0},  # FORWARD
    "AA 55 00 F2 FB": {"x": -0.10, "y": 0.0,  "yaw_rate": 0.0,  "duration": 2.0},  # BACKWARD
    "AA 55 00 F3 FB": {"x":  0.00, "y": 0.0,  "yaw_rate": 0.6,  "duration": 1.5},  # TURN LEFT (CCW)
    "AA 55 00 F4 FB": {"x":  0.00, "y": 0.0,  "yaw_rate":-0.6,  "duration": 1.5},  # TURN RIGHT (CW)
}

# --- FSM states for voice performance (11–19) ---
IDLE, FWD, BACK, TL, TR, LFWD, LBACK, LLEFT, LRIGHT = range(9)
_state = IDLE
_last_pose_tag = None

run_st = True
_motion_lock = threading.Lock()
_motion_timer = None
_drive_timer = None


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
    try:
        callable_srv(filename_with_ext)
    except TypeError:
        callable_srv(filename_with_ext, False)


def stop_motion(vel_pub):
    """Force a stop (thread-safe)."""
    global _motion_timer
    with _motion_lock:
        if _motion_timer is not None:
            _motion_timer.shutdown()
            _motion_timer = None
        vel_pub.publish(x=0.0, y=0.0, yaw_rate=0.0)


def _apply_pose(pose_pub, pose_dict, run_time_ms=300):
    pose_pub.publish(
        stance_x=pose_dict['stance_x'], stance_y=pose_dict['stance_y'],
        x_shift=pose_dict['x_shift'], height=pose_dict['height'],
        roll=pose_dict['roll'], pitch=pose_dict['pitch'], yaw=pose_dict['yaw'],
        run_time=run_time_ms
    )


def run_motion(pose_pub, gait_pub, vel_pub, m):
    """Timed motions from F1–F4"""
    global _motion_timer
    with _motion_lock:
        if _motion_timer is not None:
            _motion_timer.shutdown()
            _motion_timer = None
        pose = dict(PUPPY_POSE0)
        pose_pub.publish(**pose, run_time=400)
        gait_pub.publish(**GAIT0)
        rospy.sleep(0.1)
        vel_pub.publish(x=m["x"], y=m["y"], yaw_rate=m["yaw_rate"])

        def _auto_stop(_event):
            stop_motion(vel_pub)

        _motion_timer = rospy.Timer(rospy.Duration.from_sec(m["duration"]), _auto_stop, oneshot=True)


def _drive_cb(_event, pose_pub, vel_pub):
    """Continuous drive loop for FSM-based motion (11–19)."""
    global _state, _last_pose_tag
    if _state == IDLE:
        vel_pub.publish(x=0.0, y=0.0, yaw_rate=0.0)
        _last_pose_tag = None
        return
    if _state == FWD:
        vel_pub.publish(x=0.12, y=0.0, yaw_rate=0.0)
        return
    if _state == BACK:
        vel_pub.publish(x=-0.12, y=0.0, yaw_rate=0.0)
        return
    if _state == TL:
        vel_pub.publish(x=0.0, y=0.0, yaw_rate=+0.6)
        return
    if _state == TR:
        vel_pub.publish(x=0.0, y=0.0, yaw_rate=-0.6)
        return

    # Lean poses
    pose = dict(PUPPY_POSE0)
    tag = None
    if _state == LFWD:
        pose['pitch'] = math.radians(+15); tag = "LFWD"
    elif _state == LBACK:
        pose['pitch'] = math.radians(-15); tag = "LBACK"
    elif _state == LLEFT:
        pose['roll']  = math.radians(+15); tag = "LLEFT"
    elif _state == LRIGHT:
        pose['roll']  = math.radians(-15); tag = "LRIGHT"
    if tag and _last_pose_tag != tag:
        _apply_pose(pose_pub, pose, run_time_ms=300)
        _last_pose_tag = tag
    vel_pub.publish(x=0.0, y=0.0, yaw_rate=0.0)


def parse_and_dispatch(hex_data, pose_pub, vel_pub, gait_pub, run_ag_srv):
    """Interpret 5-byte frame and dispatch action."""
    global _state, _last_pose_tag, run_st

    # 1) Action groups
    if hex_data in FRAME_TO_ACTION:
        stop_motion(vel_pub)
        run_action_group(run_ag_srv, FRAME_TO_ACTION[hex_data])
        rospy.loginfo("ActionGroup -> %s", FRAME_TO_ACTION[hex_data])
        return

    # 2) Timed motions (F1–F4)
    if hex_data in MOTION_MAP:
        run_motion(pose_pub, gait_pub, vel_pub, MOTION_MAP[hex_data])
        rospy.loginfo("Motion -> %s", MOTION_MAP[hex_data])
        return

    # 3) Continuous FSM-based voice modes (11–19)
    if   hex_data == "AA 55 00 01 FB": _state = FWD
    elif hex_data == "AA 55 00 02 FB": _state = BACK
    elif hex_data == "AA 55 00 03 FB": _state = TL
    elif hex_data == "AA 55 00 04 FB": _state = TR
    elif hex_data == "AA 55 00 05 FB": _state = LFWD; _last_pose_tag = None
    elif hex_data == "AA 55 00 06 FB": _state = LBACK; _last_pose_tag = None
    elif hex_data == "AA 55 00 07 FB": _state = LLEFT; _last_pose_tag = None
    elif hex_data == "AA 55 00 08 FB": _state = LRIGHT; _last_pose_tag = None
    elif hex_data == "AA 55 00 09 FB":
        _state = IDLE
        stop_motion(vel_pub)
        _apply_pose(pose_pub, dict(PUPPY_POSE0), run_time_ms=300)
        rospy.loginfo("STOP -> IDLE")
        return
    elif hex_data == "AA 55 00 0A FB":
        _state = IDLE
        stop_motion(vel_pub)
        _apply_pose(pose_pub, dict(PUPPY_POSE0), run_time_ms=300)
        rospy.loginfo("QUIT received")
        run_st = False
        rospy.signal_shutdown("QUIT command")
        return

    # 4) Legacy/demo motions
    if hex_data == MARCH_CODE:
        stop_motion(vel_pub)
        pose = dict(PUPPY_POSE0)
        pose_pub.publish(**pose, run_time=500)
        rospy.sleep(0.2)
        vel_pub.publish(x=0.1, y=0.0, yaw_rate=0.0)
        rospy.sleep(2)
        vel_pub.publish(x=0.0, y=0.0, yaw_rate=0.0)

    elif hex_data == LIEDOWN_CODE:
        stop_motion(vel_pub)
        pose = dict(PUPPY_POSE0); pose['height'] = -6
        pose_pub.publish(**pose, run_time=500)

    elif hex_data == LOOKUP_CODE:
        stop_motion(vel_pub)
        pose = dict(PUPPY_POSE0); pose['pitch'] = math.radians(20)
        pose_pub.publish(**pose, run_time=500)


if __name__ == "__main__":
    print(BANNER)
    rospy.init_node('voice_interaction_demo_ag_nav')
    rospy.on_shutdown(on_shutdown)

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    rospy.sleep(0.5)

    PuppyPosePub.publish(**PUPPY_POSE0, run_time=500)
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(**GAIT0)

    rospy.wait_for_service('/puppy_control/runActionGroup', timeout=5)
    RunAG = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)

    dev = "/dev/ttyUSB1"
    ser = serial.Serial(dev, 115200, timeout=0.2)
    rospy.loginfo("Using USB: %s", dev)

    # continuous FSM driver
    _drive_timer = rospy.Timer(
        rospy.Duration(0.02),
        lambda evt: _drive_cb(evt, PuppyPosePub, PuppyVelocityPub),
        oneshot=False
    )

    while run_st and not rospy.is_shutdown():
        code = read_fixed_frame(ser)
        if code:
            rospy.loginfo("RX %s", code)
            parse_and_dispatch(code, PuppyPosePub, PuppyVelocityPub, PuppyGaitConfigPub, RunAG)
        else:
            rospy.sleep(0.02)
