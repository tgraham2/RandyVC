import os
import sys
import rospy
import math
import serial
from std_msgs.msg import *
from puppy_control.msg import Velocity, Pose, Gait

print('''
**********************************************************
******************åŠŸèƒ½:è¯­éŸ³äº¤äº’ä¾‹ç¨‹(function: voice interaction routine)*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * æŒ‰ä¸‹Ctrl+Cå¯å…³é—­æ­¤æ¬¡ç¨‹åºè¿è¡Œï¼Œè‹¥å¤±è´¥è¯·å¤šæ¬¡å°è¯•ï¼(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

# Initialize robot dog's posture and gait configuration.ï¼ˆåˆå§‹åŒ–æœºå™¨ç‹—çš„å§¿æ€å’Œæ­¥æ€é…ç½®ï¼‰
PuppyPose = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.000, 'height': -10, 'x_shift': -0.5, 'stance_x': 0, 'stance_y': 0}
GaitConfig = {'overlap_time': 0.2, 'swing_time': 0.2, 'clearance_time': 0.0, 'z_clearance': 3}

# Stop function.ï¼ˆåœæ­¢å‡½æ•°ï¼‰
run_st = True
def Stop():
    global run_st
    run_st = False
    print('å…³é—­ä¸­...')

# Parse serial data.ï¼ˆè§£æžä¸²å£æ•°æ®ï¼‰
def parse_serial_data(data):
    # Convert byte data to hexadecimal string.ï¼ˆå°†å­—èŠ‚æ•°æ®è½¬æ¢ä¸ºåå…­è¿›åˆ¶å­—ç¬¦ä¸²ï¼‰
    hex_data = ' '.join(format(byte, '02X') for byte in data)
    print(f"Received data: {hex_data}")

    # Execute corresponding actions based on different commands.ï¼ˆæ ¹æ®ä¸åŒçš„æŒ‡ä»¤æ‰§è¡Œç›¸åº”çš„åŠ¨ä½œï¼‰
    if hex_data == "AA 55 00 76 FB":  # March in place.ï¼ˆåŽŸåœ°è¸æ­¥ï¼‰
        print("æ‰§è¡ŒåŽŸåœ°è¸æ­¥")
        PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
        rospy.sleep(0.5)
        PuppyVelocityPub.publish(x=0.1, y=0, yaw_rate=0)
        rospy.sleep(2)
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)

    elif hex_data == "AA 55 00 0A FB":  # Stand at attention.ï¼ˆç«‹æ­£ï¼‰
        print("æ‰§è¡Œç«‹æ­£")
        PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    elif hex_data == "AA 55 00 0B FB":  # Lie down.ï¼ˆè¶´ä¸‹ï¼‰
        print("æ‰§è¡Œè¶´ä¸‹")
        PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-6, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)

    elif hex_data == "AA 55 00 8D FB":  # ï¼ˆæŠ¬å¤´ï¼‰
        print("æ‰§è¡ŒæŠ¬å¤´")
        PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(20), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)

    elif hex_data == "AA 55 00 09 FB":  # Stop.ï¼ˆåœæ­¢ï¼‰
        print("åœæ­¢è¯†åˆ«")
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
        global run_st
        run_st = False

if __name__ == "__main__":
    # Initialize ROS node.ï¼ˆåˆå§‹åŒ– ROS èŠ‚ç‚¹ï¼‰
    rospy.init_node('voice_interaction_demo')
    rospy.on_shutdown(Stop)

    # Initialize publisher.ï¼ˆå‘å¸ƒå™¨åˆå§‹åŒ–ï¼‰
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)

    rospy.sleep(0.5)

    # Robot dog stands.ï¼ˆæœºå™¨ç‹—ç«™ç«‹ï¼‰
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                         height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500)
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time=GaitConfig['overlap_time'], swing_time=GaitConfig['swing_time'],
                               clearance_time=GaitConfig['clearance_time'], z_clearance=GaitConfig['z_clearance'])

    ser = serial.Serial(os.getenv( "WONDERECHO_USB","ERR#"),115200, timeout=1)
    print (f"Using USB: {os.getenv('WONDERECHO_USB')}")   

    while run_st:
        # Read and parse all serial port data.ï¼ˆè¯»å–æ‰€æœ‰ä¸²å£æ•°æ®å¹¶è§£æžï¼‰
        if ser.in_waiting > 0:
            data = ser.read_all()  # Read all available data.ï¼ˆè¯»å–æ‰€æœ‰å¯ç”¨æ•°æ®ï¼‰
            parse_serial_data(data)  # Parse data.ï¼ˆè§£æžæ•°æ®ï¼‰
        rospy.sleep(0.1)
