#!/usr/bin/env python3

import roslaunch
import rospy
import time

map_path = "/home/student/catkin_ws/src/RoboticsTeam31/maps/task5_map"

rospy.init_node("task5map", anonymous=True)
ctrl_c = True
start_time =rospy.get_time()

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()



while ctrl_c:
    print(f"Saving map at time: {rospy.get_time()}...")
    node = roslaunch.core.Node(package="map_server", node_type="map_saver", args=f"-f {map_path}")
    process = launch.launch(node)
    if rospy.get_time() - start_time >= 180:
        ctrl_c = False
    time.sleep(10)