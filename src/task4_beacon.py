#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from tb3 import Tb3Move
<<<<<<< HEAD
from geometry_msgs.msg import Twist
=======
>>>>>>> 04d57d57cd02d8fd1cee53860377db42575b4075

class colour_search(object):

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
<<<<<<< HEAD
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, self.callback_lidar)
=======
>>>>>>> 04d57d57cd02d8fd1cee53860377db42575b4075
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0
<<<<<<< HEAD
        self.cvbridge_interface 

        self.vel = Twist()
=======
>>>>>>> 04d57d57cd02d8fd1cee53860377db42575b4075

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 10000

<<<<<<< HEAD
        # Thresholds for ["Blue", "Red", "Green", "Turquoise", "Purple", "Yellow"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100), (148, 150, 100), (25, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255), (150, 250, 255), (30, 190, 255)]
=======
        # Thresholds for ["Blue", "Red", "Green", "Turquoise", "Purple"]
        # Threshold values received by looking at each pillar then generating the image (the rqt_image_view one) and its own plot
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100), (148, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255), (150, 250, 255)]
>>>>>>> 04d57d57cd02d8fd1cee53860377db42575b4075

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
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        self.stop_counter = 20 #30
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
                
            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment (blob size = {self.m00:.0f}), scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

            elif self.move_rate == 'slow':
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

            elif self.move_rate == 'stop' and self.stop_counter > 0:
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
<<<<<<< HEAD
                self.robot_controller.set_move_cmd(0.0, self.move_forward)
=======
                self.robot_controller.set_move_cmd(0.0, 0.0)
>>>>>>> 04d57d57cd02d8fd1cee53860377db42575b4075

            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()
            self.rate.sleep()
<<<<<<< HEAD

    def move_forward(self):
        self.vel.linear.x = 0.3
=======
>>>>>>> 04d57d57cd02d8fd1cee53860377db42575b4075
            
if __name__ == '__main__':
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass
