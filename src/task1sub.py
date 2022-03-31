#!/usr/bin/env python3

#!/usr/bin/env python3
# A simple ROS subscriber node in Python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Subscriber():

    def callback(self, odom_data):
        orientation_x = odom_data.pose.pose.orientation.x
        orientation_y = odom_data.pose.pose.orientation.y
        orientation_z = odom_data.pose.pose.orientation.z
        orientation_w = odom_data.pose.pose.orientation.w
        (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                                orientation_y, orientation_z, orientation_w],
                                'sxyz')

        linear_x = odom_data.pose.pose.position.x
        linear_y = odom_data.pose.pose.position.y                        
        print("x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(linear_x, linear_y, yaw))

    def __init__(self):
        self.node_name = "odem_subscriber"
        topic_name = "odom"

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber(topic_name, Odometry, self.callback)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()