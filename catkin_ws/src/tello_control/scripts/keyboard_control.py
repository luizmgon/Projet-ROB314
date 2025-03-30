#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pynput import keyboard

# Initialize ROS node
rospy.init_node('keyboard_controller_node')

# Publisher to send control commands to tello_driver
control_pub = rospy.Publisher('/tello/control', String, queue_size=10)

def on_press(key):
    print(f'Key {str(key)} pressed')

    if str(key) == "'t'":  # Takeoff
        print("Taking off...")
        control_pub.publish("takeoff")
    elif str(key) == "'l'":  # Land
        print("Landing...")
        control_pub.publish("land")
    elif str(key) == 'Key.esc':  # ESC key
        control_pub.publish("finish")
        rospy.loginfo("Exiting keyboard controller node...")
        rospy.signal_shutdown("Shutting down keyboard controller")


# Start listening for keyboard input
listener = keyboard.Listener(on_press=on_press)
listener.start()

# Spin to keep the node running
rospy.spin()
