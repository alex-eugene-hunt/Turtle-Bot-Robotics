#!/usr/bin/env python

# Alex Hunt, Averi Bates, Aminat Oyeleke
# CS-4023-001 : Professor Hougen
# Project 2 - Reactive Robotics using ROS and TurtleBots
# control_node.py - This code controls behaviors 1 - 6

# ----- REFERENCES -----
# We heavily used our project 1 code
# http://wiki.ros.org/turtlebot_teleop
# https://github.com/turtlebot/turtlebot - source code for turtlebot teleop code
# http://library.isr.ist.utl.pt/docs/roswiki/turtlebot_teleop(2f)Tutorials(2f)Teleoperation.html
# https://github.com/Kichlids/
# https://github.com/yasharAhari/
# https://www.youtube.com/watch?v=STSPDYFDE8E
# https://learn.turtlebot.com/2015/02/03/7/
# https://classes.cs.uchicago.edu/archive/2022/spring/20600-1/lab_a.html

import rospy
import random
import math
import numpy
import time

from robot_msgs.msg import keyboard
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan

# Constant variables
# Speed in feet per second
LIN_SPD = 0.75
# Rotation speed in feet per second
ANG_SPD = 0.5
# Distance in feet to move forward in move foward behavior (6)
AUTO_FORWARD = 1
# The obstacle avoidance threshold in feet, considering the position of the laser scan sensor
LASER_AVOID_DIST_F = 2

# Subscribes to keyboard input and configures
# velocity message (vel_msg) AND whether robot is controlled by user (is_teleop_controlled)
class Keyboard():
    def __init__(self):
        self.support = Support()
        self.is_teleop_controlled = False
        self.vel_msg = Twist()
        self.keyboard_sub = rospy.Subscriber('/robot/keyboard_input', keyboard, self.keyboard_callback)
	self.leader_command_entered = False

    # Callback function for handling keyboard input
    def keyboard_callback(self, data):
        self.vel_msg = Twist()
        if data.is_teleop:
            if data.command == 'w':
                self.vel_msg.linear.x = self.support.feet_to_meters(LIN_SPD)
            elif data.command == 'a':
                self.vel_msg.angular.z = self.support.feet_to_meters(ANG_SPD)
            elif data.command == 's':
                self.vel_msg.linear.x = self.support.feet_to_meters(-LIN_SPD)
	    elif data.command == 'f':
		self.leader_command_entered = True
            else:
                self.vel_msg.angular.z = self.support.feet_to_meters(-ANG_SPD)
        self.is_teleop_controlled = data.is_teleop

# Subscribes to the bumper sensor and configures whether collision has been detected (collision_detected)
class Bumper():
    def __init__(self):
        self.collision_detected = False
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
    # callback function for bumper
    def bumper_callback(self, data):
        if data.state == BumperEvent.PRESSED:
            self.collision_detected = True

# Subscribes to the laser scanner sensor and configures:
# - detection of symmetric obstacles (symmetric_obstacle_detected)
# - detection of asymmetric obstacles (asymmetric_obstacles_detected)
# Also, stores the received LaserScan data and the minimum index of the minimum distance.
class Laser():

    def __init__(self):
        self.support = Support()
        self.symmetric_obstacle_detected = False
        self.asymmetric_obstacle_detected = False
        self.laser_data = LaserScan()
        self.laser_min_index = 0
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
    # Retrieve the minimum distance and its corresponding index from the range values
    def find_min_laser_data(self, array):
        min_val = 400
        min_index = 0
        for i in range(len(array)):
            if math.isnan(i) == False and min_val > array[i]:
                min_val = array[i]
                min_index = i
        return min_val, min_index

    # Callback function for Laser Scanner
    def laser_callback(self, data):

        # Convert ranges from meters to feet
        ranges = []
        for i in range(len(data.ranges)):
            ranges.append(self.support.meters_to_feet(data.ranges[i]))

        min_val, min_index = self.find_min_laser_data(ranges)

        if min_val < LASER_AVOID_DIST_F:

            i = len(data.ranges) - 1 - min_index
            val_i = self.support.meters_to_feet(data.ranges[i])
            # This will see it both sides of the object are about the same length away
            # If so, it is considered symmetric
            if abs(min_val - val_i) < LASER_AVOID_DIST_F:
                self.symmetric_obstacle_detected = True
                self.asymmetric_obstacle_detected = False
            # If not, it is not considered symmetric
            else:
                self.symmetric_obstacle_detected = False
                self.asymmetric_obstacle_detected = True
        # Object not detected
        else:
            self.symmetric_obstacle_detected = False
            self.asymmetric_obstacle_detected = False

        self.laser_data = data
        self.laser_min_index = min_index


# Support class contains helper functions for other classes to utilize
class Support():

    # Convert meters to feet
    def meters_to_feet(self, val):
        return val * 3.28

    # Convert feet to meters
    def feet_to_meters(self, val):
        return val / 3.28

    # Convert radians to degrees
    def rad_to_deg(self, rad):
        return rad * 180 / math.pi

    # Return a random number in a range
    def get_random_number(self, min, max):
        return random.uniform(min, max)

# Class publishes velocity messages
# as well as contain routine to be performed by robot
class Movement():

    def __init__(self, keyboard, laser):
        self.keyboard = keyboard
        self.laser = laser
        self.support = Support()

        self.velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)

    # Move robot with message
    def velocity_publish(self, vel_msg):
        self.velocity_pub.publish(vel_msg)

    # Performs forward movement and random turning
    def autonomous(self):

        # Moving forward
        move_msg = Twist()
        move_msg.linear.x = self.support.feet_to_meters(LIN_SPD)

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        print('Moving forward 1ft')

        while (current_distance < AUTO_FORWARD):
            if self.keyboard.is_teleop_controlled or self.laser.asymmetric_obstacle_detected or self.laser.symmetric_obstacle_detected:
                return

            self.velocity_publish(move_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = LIN_SPD * (t1 - t0)

        # Rotating
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        target_angle = random.choice([-21.0, 21.0])

        print('Rotating ' + str(round(target_angle, 2)))

        # Determine the direction to turn
        turn_msg = Twist()
        if target_angle >= 0:
            turn_msg.angular.z = ANG_SPD
        else:
            turn_msg.angular.z = -ANG_SPD


        while (current_angle < abs(target_angle)):
            if self.keyboard.is_teleop_controlled or self.laser.asymmetric_obstacle_detected or self.laser.symmetric_obstacle_detected:
                return

            self.velocity_publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.support.rad_to_deg(ANG_SPD) * (t1 - t0)

    # Turn 180 degrees
    def escape(self):
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        target_angle = 250 # this is not coded as 180 due to the turn speed, we found this was our best

        turn_msg = Twist()
        turn_msg.angular.z = ANG_SPD

        while (current_angle < abs(target_angle)):
            if self.keyboard.is_teleop_controlled:
               return

            self.velocity_publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.support.rad_to_deg(ANG_SPD) * (t1 - t0)

    # Turn until asymmetric obstacle is not visible
    def avoid(self):

        turn_msg = Twist()

        # Turn in the direction away from the closest obstacle
        if self.laser.laser_min_index < (len(self.laser.laser_data.ranges) - 1) / 2:
            turn_msg.angular.z = ANG_SPD
        else:
            turn_msg.angular.z = -ANG_SPD

        while self.laser.asymmetric_obstacle_detected:
            if self.keyboard.is_teleop_controlled or self.laser.symmetric_obstacle_detected:
                return

            self.velocity_publish(turn_msg)

    def follow(self):
        print("Starting follow mode...")
        start_time = time.time()

        while time.time() - start_time < 30:  # Follow for 30 seconds

            if self.laser.laser_min_index < len(self.laser.laser_data.ranges):
                measured_distance = self.support.meters_to_feet(self.laser.laser_data.ranges[self.laser.laser_min_index])
                if 3 <= measured_distance <= 5:  # Object between 3 - 5 feet

		        # Determine the relative position of the object
		        target_index = self.laser.laser_min_index
		        center_index = len(self.laser.laser_data.ranges) // 2

		        # Calculate the angle to the object relative to the robot's center
		        angle_to_object = (target_index - center_index) * self.laser.laser_data.angle_increment

		        # Implement movement and turning towards the object
		        move_msg = Twist()
		        move_msg.linear.x = 0.2  # Set linear speed to move forward

		        # Calculate the turning speed based on the angle to the object
		        turn_speed = 0.9 * angle_to_object  # Adjust this factor for better turning behavior

		        move_msg.angular.z = turn_speed  # Set angular speed for turning
		        self.velocity_publish(move_msg)

        print("Follow mode ended after 30 seconds.")



def init_control_node():

    rospy.init_node('control_node', anonymous = False)
    rate = rospy.Rate(100)

    # Initialize all publishers and subscribers
    keyboard = Keyboard()
    bumper = Bumper()
    laser = Laser()

    movement = Movement(keyboard, laser)

    while not rospy.is_shutdown():

	if keyboard.leader_command_entered:
		print('Leader command entered!')
		movement.follow()
		keyboard.leader_command_entered = False

        # Halt if collision(s) detected by bumper(s).
        if bumper.collision_detected:
            print('\rCOLLISION DETECTED: HALT! ')
            vel_msg = Twist()
            movement.velocity_publish(vel_msg)

        # Accept keyboard movement commands from a human user.
        elif keyboard.is_teleop_controlled and keyboard.leader_command_entered == False:
            print('\rUSER CONTROL DETECTED! ')
            movement.velocity_publish(keyboard.vel_msg)

        # Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
        elif laser.symmetric_obstacle_detected:
            print('\rENCOUNTERED SYMMETRIC OBSTACLE! ')
            movement.escape()

        # Avoid asymmetric obstacles within 1ft in front of the robot.
        elif laser.asymmetric_obstacle_detected:
            print('\rENCOUNTERED ASYMMETRIC OBSTACLE! ')
            movement.avoid()

        # Turn randomly (uniformly sampled within 15 degrees) after every 1ft of forward movement. AND Drive forward.
        else:
            movement.autonomous()

        rate.sleep()

if __name__ == '__main__':
    try:
        init_control_node()
    except rospy.ROSInterruptException:
        pass
