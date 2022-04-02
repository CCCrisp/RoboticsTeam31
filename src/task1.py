#!/usr/bin/env python3

from itertools import filterfalse
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

class Square:
    def callback_function(self, odom_data):
        # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y
        print(f"x = '{pos_x:.3f}', y = '{pos_y:.3f}', theta_z = '{or_w:.3f}'")

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        node_name = "move_circle"
        
        self.startup = True
        self.turn = False

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(1) # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True

    def main_loop(self):
        wait = 0
        while not self.ctrl_c:
            if self.startup:
                self.vel = Twist()
            elif self.turn:
                if abs(self.x0 - self.x) <= 0.055 and wait > 4:
                    # If the robot has turned 90 degrees (in radians) then stop turning
                    #self.vel.linear.x = 0
                    #self.vel.angular.z = 0 # rad/s
                    self.ctrl_c = True
                else:
                    self.vel = Twist()
                    path_rad = 0.5 # m
                    lin_vel = 0.1 # m/s
                    wait += 1
                    
                    self.vel.linear.x = lin_vel
                    self.vel.angular.z = -(lin_vel / path_rad) # rad/s
            else:
                if abs(self.x0 - self.x) <= 0.008 and wait > 4:
                    # if distance travelled is greater than 0.5m then stop, and start turning:
                    self.vel = Twist()
                    self.turn = True
                    # self.vel.linear.x = 0
                    # self.vel.angular.z = 0 # rad/s
                    self.x0 = self.x
                    self.y0 = self.y
                    wait = 0
                else:
                    self.vel = Twist()
                    path_rad = 0.5 # m
                    lin_vel = 0.1 # m/s
                    wait += 1

                    self.vel.linear.x = lin_vel
                    self.vel.angular.z = lin_vel / path_rad # rad/s
            self.pub.publish(self.vel)
            self.rate.sleep()
            
if __name__ == '__main__':
    movesquare_instance = Square()
    try:
        movesquare_instance.main_loop()
    except rospy.ROSInterruptException:
        pass