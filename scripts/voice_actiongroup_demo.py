#!/usr/bin/env python3
# coding=utf8
import os
import sys
import serial
import signal
import rospy
import pyttsx3
from puppy_control.srv import SetRunActionName

# ----------------------------------------------------
# Configuration
# ----------------------------------------------------
ACTION_GROUP_DIR = "/home/ubuntu/software/puppypi_control/ActionGroups"
BAUD = 115200

# ----------------------------------------------------
# Voice command → ActionGroup map (PERFORM-* names)
# ----------------------------------------------------
FRAME_MAP = {
    "AA 55 00 80 FB": ("PERFORM-1",              "1.d6a"),
    "AA 55 00 81 FB": ("PERFORM-2-LEGS-STAND",   "2_legs_stand.d6ac"),
    "AA 55 00 82 FB": ("PERFORM-BOW",            "bow.d6ac"),
    "AA 55 00 83 FB": ("PERFORM-BOXING",         "boxing.d6ac"),
    "AA 55 00 84 FB": ("PERFORM-BOXING2",        "boxing2.d6ac"),
    "AA 55 00 85 FB": ("PERFORM-DEMO",           "demo.d6ac"),
    "AA 55 00 86 FB": ("PERFORM-GRAB",           "grab.d6a"),
    "AA 55 00 87 FB": ("PERFORM-JUMP",           "jump.d6ac"),
    "AA 55 00 88 FB": ("PERFORM-KICK-LEFT",      "kick_ball_left.d6ac"),
    "AA 55 00 89 FB": ("PERFORM-KICK-RIGHT",     "kick_ball_right.d6ac"),
    "AA 55 00 A1 FB": ("PERFORM-LIE-DOWN",       "lie_down.d6ac"),
    "AA 55 00 A2 FB": ("PERFORM-LOOK-DOWN",      "look_down.d6ac"),
    "AA 55 00 A3 FB": ("PERFORM-MOONWALK",       "moonwalk.d6ac"),
    "AA 55 00 A4 FB": ("PERFORM-NOD",            "nod.d6ac"),
    "AA 55 00 A5 FB": ("PERFORM-PEE",            "pee.d6ac"),
    "AA 55 00 A6 FB": ("PERFORM-PRESS-UP",       "press-up.d6ac"),
    "AA 55 00 A7 FB": ("PERFORM-PUSH-UP",        "push-up.d6ac"),
    "AA 55 00 A8 FB": ("PERFORM-SHAKE-HANDS",    "shake_hands.d6ac"),
    "AA 55 00 A9 FB": ("PERFORM-SHAKE-HEAD",     "shake_head.d6ac"),
    "AA 55 00 AA FB": ("PERFORM-SIT",            "sit.d6ac"),
    "AA 55 00 AB FB": ("PERFORM-SPACEWALK",      "spacewalk.d6ac"),
    "AA 55 00 AC FB": ("PERFORM-STAND",          "stand.d6ac"),
    "AA 55 00 AD FB": ("PERFORM-STRETCH",        "stretch.d6ac"),
    "AA 55 00 AE FB": ("PERFORM-UP-STAIRS-2",    "up_stairs_2cm.d6ac"),
    "AA 55 00 AF FB": ("PERFORM-UP-STAIRS-3",    "up_stairs_3.5cm.d6ac"),
    "AA 55 00 B0 FB": ("PERFORM-WAVE",           "wave.d6ac"),
    "AA 55 00 B1 FB": ("PERFORM-PUSH-UP01",      "push-up01.d6ac"),
    "AA 55 00 B2 FB": ("PERFORM-STAND-WITH-ARM", "stand_with_arm.d6a"),
    "AA 55 00 B3 FB": ("PERFORM-CLAMPING",       "Clamping.d6a"),
    "AA 55 00 B4 FB": ("PERFORM-ARM-TEST",       "arm_test.d6a"),
}

# ----------------------------------------------------
# Globals
# ----------------------------------------------------
run_st = True

# ----------------------------------------------------
# Utilities
# ----------------------------------------------------
def bytes_to_hex_string(data: bytes) -> str:
    return ' '.join(f"{b:02X}" for b in data)

def speak_acknowledge():
    """Simple audible feedback: 'Copy that'."""
    try:
        engine.say("Copy that")
        engine.runAndWait()
    except Exception as e:
        print(f"(TTS error) {e}")

def stop_all():
    print("Stopping all operations...")
    try:
        runActionGroup_srv('stand.d6ac', True)
    except Exception:
        pass
    rospy.signal_shutdown("Program exited cleanly")

def signal_handler(sig, frame):
    stop_all()

# ----------------------------------------------------
# Core logic
# ----------------------------------------------------
def handle_voice_command(hex_data: str):
    """Run action group corresponding to the received frame."""
    if hex_data not in FRAME_MAP:
        print(f"Unknown frame: {hex_data}")
        return

    label, filename = FRAME_MAP[hex_data]
    print(f"→ {label}  |  file: {filename}")
    speak_acknowledge()

    full_path = os.path.join(ACTION_GROUP_DIR, filename)
    if not os.path.exists(full_path):
        print(f"Missing file: {full_path}")
        return

    try:
        runActionGroup_srv(filename, True)
        print(f"Executed {filename}")
    except rospy.ServiceException as e:
        print(f"Failed to execute {filename}: {e}")

# ----------------------------------------------------
# Main
# ----------------------------------------------------
if __name__ == "__main__":
    # --- Signal handling ---
    signal.signal(signal.SIGINT, signal_handler)

    # --- Initialize ROS ---
    rospy.init_node('voice_actiongroup_demo', disable_signals=True)
    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
    rospy.sleep(0.3)

    # --- Initialize TTS ---
    engine = pyttsx3.init('espeak')
    engine.setProperty('voice', 'gmw/en-us')
    engine.setProperty('rate', 165)

    # --- Auto-detect serial port (ttyUSB0 or ttyUSB1) ---
    candidate_ports = ["/dev/ttyUSB0", "/dev/ttyUSB1"]
    voice_port = None
    for p in candidate_ports:
        if os.path.exists(p):
            voice_port = p
            break

    if voice_port is None:
        print("❌ No voice module detected (neither /dev/ttyUSB0 nor /dev/ttyUSB1).")
        sys.exit(1)

    os.environ["WONDERECHO_USB"] = voice_port  # set for future use
    print(f"✅ Using voice module on {voice_port}")

    try:
        ser = serial.Serial(voice_port, BAUD, timeout=1)
    except Exception as e:
        print(f"❌ Failed to open {voice_port}: {e}")
        sys.exit(1)

    print("Voice → ActionGroup controller ready.")
    print(f"Action groups path: {ACTION_GROUP_DIR}")

    # --- Main loop ---
    while not rospy.is_shutdown() and run_st:
        if ser.in_waiting > 0:
            data = ser.read_all()
            hex_data = bytes_to_hex_string(data)
            print(f"Received frame: {hex_data}")
            handle_voice_command(hex_data)
        rospy.sleep(0.1)
