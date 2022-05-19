#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
from http.client import ImproperConnectionState
import rospy
import actionlib

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pathlib import Path
from sensor_msgs.msg import Image

# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow
import numpy as np
import time

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()
    cvbridge_interface = CvBridge()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        self.ctrl_c = False
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
        self.start_time = rospy.get_time()

        
        # Thresholds for ["Blue", "Red", "Green", "Yellow"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (25, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (30, 190, 255)]
    
    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:5]
        right_arc = scan_data.ranges[-5:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]

    
    def action_server_launcher(self, goal: SearchGoal):
        r = rospy.Rate(10)

        success = True

        # Get the current robot odometry:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy

        
        # set the robot velocity:
        turning = 0
        self.vel_controller.set_move_cmd(goal.fwd_velocity, turning)
        self.vel_controller.publish()

        self.tb3_lidar.TurnLeft = False
        while success:
            if self.tb3_lidar.min_distance > goal.approach_distance:
                turning = 0
                goal.fwd_velocity = 0.1
                self.vel_controller.set_move_cmd(goal.fwd_velocity, turning)
                self.vel_controller.publish()
                
            elif self.tb3_lidar.TurnLeft: 
                turning = 0.2
                goal.fwd_velocity = 0.0
                self.vel_controller.set_move_cmd(goal.fwd_velocity, turning)
                self.vel_controller.publish()
                time.sleep(1)

            else :
                turning = -0.2
                goal.fwd_velocity = 0.0
                self.vel_controller.set_move_cmd(goal.fwd_velocity, turning)
                self.vel_controller.publish()
                time.sleep(1)
            
            
            # cancel if the time has elapsed
            if rospy.get_time() - self.start_time >= 10:
                break
            
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
            # populate the feedback message and publish it:
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)


        rospy.loginfo("time has elapsed")
        self.result.total_distance_travelled = self.distance
        self.result.closest_object_distance = self.tb3_lidar.min_distance
        self.result.closest_object_angle = self.tb3_lidar.closest_object_position
        self.actionserver.set_succeeded(self.result)
        self.vel_controller.stop()


if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()