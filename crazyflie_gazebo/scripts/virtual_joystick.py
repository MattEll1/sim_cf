#!/usr/bin/env python3
# filepath: /code/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts/auto_init.py

import rospy
from sensor_msgs.msg import Joy
import time

# Initialize ROS node
rospy.init_node('auto_initializer', anonymous=True)

# Create publisher
joy_pub = rospy.Publisher('/cf1/joy', Joy, queue_size=10)

# Wait for publisher to connect
time.sleep(1.0)
print("Publisher ready. Starting initialization sequence...")

# Create base Joy message
joy_msg = Joy()
joy_msg.axes = [0.0] * 9  # Make sure we have enough axes
joy_msg.buttons = [0] * 11  # Make sure we have enough buttons

def send_button_press(button_index, duration=0.5):
    """Press and release a button with proper timing"""
    # All buttons off
    for i in range(len(joy_msg.buttons)):
        joy_msg.buttons[i] = 0
    
    # Press button
    joy_msg.buttons[button_index] = 1
    joy_pub.publish(joy_msg)
    print(f"Pressed button {button_index}")
    rospy.sleep(duration)
    
    # Release button
    joy_msg.buttons[button_index] = 0
    joy_pub.publish(joy_msg)
    print(f"Released button {button_index}")
    rospy.sleep(0.5)  # Wait between button presses

# Button indices based on your joy_launch.launch
START_BUTTON = 10
A_BUTTON = 1
RB_BUTTON = 6

# 1. Press Start to arm
print("Pressing START button to arm...")
send_button_press(START_BUTTON)

# 2. Press A to select attitude control
print("Pressing A button to select attitude control...")
send_button_press(A_BUTTON)

# 3. Press RB to activate attitude control
print("Pressing RB button to activate attitude control...")
send_button_press(RB_BUTTON)

# Print success message
print("Initialization sequence completed!")

# Keep the node running to maintain the connection
rospy.spin()