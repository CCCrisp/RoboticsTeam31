#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np

class Tb3Move(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()

class Tb3Odometry(object):
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Tb3LaserScan(object):
    def laserscan_cb(self, scan_data):
        left_arc = scan_data.ranges[0:25]
        right_arc = scan_data.ranges[-25:]
        left_middle_arc = scan_data.ranges[80:100]
        right_middle_arc = scan_data.ranges[-100:-80]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.TurnLeft = False
        self.TurnLeft1 = False

        self.min_distance = front_arc.min()
        self.max_left_distance = np.array(left_middle_arc).max()
        self.max_right_distance = np.array(right_middle_arc).max()
        self.min_left_distance = np.array(left_middle_arc).min()
        self.min_right_distance = np.array(right_middle_arc).min()

        if self.min_left_distance < 0.3:
            self.TurnLeft1 = False
        elif self.min_right_distance < 0.3:
            self.TurnLeft1 = True
        
        if self.max_left_distance > self.max_right_distance :
            self.TurnLeft1 = True
        elif self.max_left_distance > self.max_right_distance :
            self.TurnLeft1 = False
        
        
        if self.min_distance in right_arc:
            self.TurnLeft = True
        else:
            self.TurnLeft = False
        # arc_angles = np.arange(-4, 46)
        # self.closest_object_position = arc_angles[np.argmin(front_arc)]

    def __init__(self):
        self.min_distance = 0.0
        self.closest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb) 