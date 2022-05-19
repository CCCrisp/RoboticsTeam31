#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:

import rospy
import argparse

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from pathlib import Path

class colour_search(object):

    cvbridge_interface = CvBridge()
    def __init__(self):
        
        node_name = "task5camera2"
        rospy.init_node(node_name,anonymous=True)

        
        cli = argparse.ArgumentParser(description=f"Command-line interface for the '{node_name}' node.")
        cli.add_argument("target_colour", metavar="COL", default="Blue", help="The name of a colour (for example)")
        
        self.args = cli.parse_args(rospy.myargv())

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image, self.camera_callback)

        self.cvbridge_interface = CvBridge()

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 10000

        self.ctrl_c = False

        # Thresholds for ["Blue", "Red", "Green", "Yellow"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (25, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (30, 190, 255)]

    def shutdown_ops(self):
        
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # create a single mask to accommodate all four dectection colours:
        
        for i in range(4):
            if i == 0:
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            else:
                mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        
        # lower = (115, 224, 100)
        # upper = (130, 255, 255)
        # mask = cv2.inRange(hsv_img, lower, upper)
        # res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        #cv2.imshow('cropped image', crop_img)
        #cv2.waitKey(0)
        if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    colour_search.show_and_save_image(cv_img, "the_beacon")

    def show_and_save_image(img, img_name):
        base_image_path = Path("home/student/catkin_ws/src/RoboticsTeam31/snaps")
        full_image_path = base_image_path.joinpath(f"{img_name}.jpg")

        cv2.imshow(img_name, img)
        cv2.waitKey(1)

        cv2.imwrite(str(full_image_path), img)
        print(f"Saved an image to '{full_image_path}'\n"
            f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
            f"file size = {full_image_path.stat().st_size} bytes")


    def main(self):
        while not self.ctrl_c:
            
            rospy.Subscriber("/camera/rgb/image_raw",Image, self.camera_callback)
            self.rate.sleep()

        

if __name__ == '__main__':
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass