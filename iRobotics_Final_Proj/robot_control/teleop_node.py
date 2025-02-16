#!/usr/bin/env python

# Alex Hunt, Averi Bates, Aminat Oyeleke
# CS-4023-001 : Professor Hougen
# Project 2 - Reactive Robotics using ROS and TurtleBots
# teleop_node.py - This code implements behavior 2

# ----- REFERENCES -----
# This is an exact copy of our project 1 code
# http://wiki.ros.org/turtlebot_teleop
# https://github.com/turtlebot/turtlebot - source code for turtlebot teleop code
# http://library.isr.ist.utl.pt/docs/roswiki/turtlebot_teleop(2f)Tutorials(2f)Teleoperation.html
# https://github.com/Kichlids/
# https://github.com/yasharAhari/

import rospy
import sys
import select
import termios
import tty

from robot_msgs.msg import keyboard

# The class accepts input from the user and broadcasts messages related to movement.
class KeyboardInput:
    
    def __init__(self):
        self.input_keys = ['w', 'a', 's', 'd','f']
        self.keyboard_pub = rospy.Publisher('/robot/keyboard_input', keyboard, queue_size = 10)
        self.construct_keyboard_msg()
    
    # From turtlebot teleop code reference above
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        return key

    # Detect the pressed key and broadcast corresponding key information.
    def construct_keyboard_msg(self):
        key = self.get_key()
        keyboard_msg = keyboard()
        
        if key in self.input_keys:
            keyboard_msg.is_teleop = True
            keyboard_msg.command = key
        else:
            keyboard_msg.is_teleop = False
        
        self.keyboard_pub.publish(keyboard_msg)

# Initialize the teleop node and the keyboard publisher.
def init_teleop_node():
    rospy.init_node('teleop_node', anonymous = False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        keyboard_input = KeyboardInput()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        init_teleop_node()
    except rospy.ROSInterruptException:
        pass
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
