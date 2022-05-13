#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion

class Task3Maze():

    def __init__(self):
        self.node_name = "navigate_maze"

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)           
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        # self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.callback_function)

        rospy.init_node(self.node_name)

        self.rate = rospy.Rate(10) # hz
        self.vel = Twist()

        self.front_distance = 0
        self.right_distance = 0
        self.left_distance = 0
        self.object_angle = 0
        self.ctrl_c = False

        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

        rospy.on_shutdown(self.shutdownhook) 
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")


    def shutdownhook(self):  
        self.vel.linear.x = 0.0 # m/s
        self.vel.angular.z = 0.0 # rad/s

        print("Stopping the robot")

        # publish to the /cmd_vel topic to make the robot stop
        self.velocity_publisher.publish(self.vel)

        self.ctrl_c = True
        

    def main_loop(self):
        dist = 0.3
        left_dist = 0.3
        right_dist = 0.3
        
        while not self.ctrl_c:
            # while self.object_angle != 0:
            #     self.turn_robot(self.object_angle)
            # self.move_forward()
            
            # self.velocity_publisher.publish(self.vel)
            # self.rate.sleep()
            
            
            
            while self.object_angle != 0:
                # print(self.max_distance)
                # print(self.object_angle)
                if self.object_angle > 0:
                    print("Turning right")
                    self.turn_right()
                else:
                    print("Turning left")
                    self.turn_left() 
            
                print("Moving forward")
                self.move_forward()

                self.velocity_publisher.publish(self.vel)
                self.rate.sleep()


            # # If there is no wall close to your front, left and right then find a wall
            # if self.front_distance > dist and self.left_distance > left_dist and self.right_distance > right_dist:
            #     print("Finding wall 1")
            #     self.find_wall()
            #     # self.new_angle = 0
            #     # while self.new_angle != 0:
            #     #     self.turn_right()
            #     #     self.new_angle = self.object_angle
            #     # self.move_forward()
            
            # # If there only is a wall close enough to the right but not the front or the left then follow the wall
            # elif self.front_distance > dist and self.left_distance > left_dist and self.right_distance < right_dist:
            #     print("Following wall 2")
            #     self.follow_wall()
            
            # # If there only is a wall close enough to the left but not the front and the right then find a wall
            # elif self.front_distance > dist and self.left_distance < left_dist and self.right_distance > right_dist:
            #     print("Turning right 3")
            #     self.turn_right()

            # # If there is a wall close to your left and right but not the front then follow the right wall by moving forwards
            # elif self.front_distance > dist and self.left_distance < left_dist and self.right_distance < right_dist:
            #     print("Following wall 4")
            #     self.follow_wall()
                
            # # If there is a wall close to your front but not the left or the right then turn left to then have a wall to the right  
            # elif self.front_distance < dist and self.left_distance > left_dist and self.right_distance > right_dist:
            #     print("Turning left 5")
            #     self.turn_left()

            # # If there is a wall close to your front and right but not the left then turn left
            # elif self.front_distance < dist and self.left_distance > left_dist and self.right_distance < right_dist:
            #     print("Turning left 6")
            #     # print("Front distance at")
            #     # print(self.front_distance)
            #     self.turn_left()
                
            # # If there is a wall close to the front and left but not the right then turn right? or left?
            # elif self.front_distance < dist and self.left_distance < left_dist and self.right_distance > right_dist:
            #     # print("Turning left 7")
            #     print("Turning right 7")
            #     self.turn_right()
                
            # # If there is a wall close to the front and left and right then turn around? or left?
            # elif self.front_distance < dist and self.left_distance < dist and self.right_distance < dist:
            #     print("Turning right 8")
            #     self.turn_right()

            # self.velocity_publisher.publish(self.vel)
            # self.rate.sleep()

    

    def callback_lidar(self, lidar_data):
        # Obtain a subset of the LaserScan.ranges array corresponding to a +/-10 degree arc in front of it. Convert this subset to a numpy array to allow for more advanced processing.

        left_arc = lidar_data.ranges[0:45]
        # left_arc = lidar_data.ranges[90]
        right_arc = lidar_data.ranges[-45:]
        # right_arc = lidar_data.ranges[-90]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()

        arc_angles = np.arange(-45, 45)
        self.object_angle = arc_angles[np.argmax(front_arc)]

        # find the miniumum object distance within the frontal laserscan arc:
        self.front_distance = np.array(front_arc).min()
        self.left_distance = np.array(left_arc).min()
        self.right_distance = np.array(right_arc).min()

        self.max_front_distance = np.array(front_arc).max()
        self.max_left_distance = np.array(left_arc).max()
        self.max_right_distance = np.array(right_arc).max()

        self.max_distance = max(self.max_front_distance, self.max_left_distance, self.max_right_distance)
        # print("Angle to turn at ")
        # print(self.object_angle)


    def find_wall(self): 
        self.vel = Twist()
        self.vel.linear.x = 0.1
        # self.vel.angular.z = -0.1


    def turn_left(self):
        self.vel = Twist()
        # self.vel.linear.x = 0
        self.vel.angular.z = 0.3


    def turn_right(self):
        self.vel = Twist()
        # self.vel.linear.x = 0
        # self.vel.linear.x = -0.1
        self.vel.angular.z = -0.3

    
    def turn_back(self):
        self.vel = Twist()
        self.vel.linear.x = -0.2
        self.vel.angular.z = 0.2
        self.find_wall()
        

    def follow_wall(self):
        self.vel = Twist()
        self.vel.linear.x = 0.2

    def move_forward(self):
        self.vel = Twist()
        self.vel.angular.z = 0
        self.vel.linear.x = 0.3

    # def turn_robot(self, angle):
        # self.vel = Twist()
        



if __name__ == '__main__':
    maze_instance = Task3Maze()
    try:
        maze_instance.main_loop()
    except rospy.ROSInterruptException:
        pass






# def callback_function(self, odom_data):
#     or_x = odom_data.pose.pose.orientation.x
#     or_y = odom_data.pose.pose.orientation.y
#     or_z = odom_data.pose.pose.orientation.z
#     or_w = odom_data.pose.pose.orientation.w

#     pos_x = odom_data.pose.pose.position.x
#     pos_y = odom_data.pose.pose.position.y
#     pos_z = odom_data.pose.pose.position.z

#     (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
