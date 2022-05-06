#! /usr/bin/env python3

from curses.ascii import ctrl
import rospy
import actionlib
import roslaunch

from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class action_client(object):
   
    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        if self.i < 100:
            self.i += 1
        else:
            self.i = 0

    def __init__(self):
        
        self.ctrl_c = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)
        self.goal = SearchGoal()
        self.client = actionlib.SimpleActionClient("/search_action_server", 
                    SearchAction)
        self.client.wait_for_server()
        rospy.on_shutdown(self.shutdown_ops)
        self.distance = 0.0
        self.i = 0

    def shutdown_ops(self):
        self.client.cancel_goal()
        self.ctrl_c = True
            
            
    def send_goal(self, velocity, approach):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        package = 'map_server'
        exe = 'map_saver'
        node = roslaunch.core.Node(package, exe, args='home/student/catkin_ws/src/rob/maps/ -f task5_map')
        self.send_goal(velocity = 0.1, approach = 0.35)
        #while not self.ctrl_c:
        #    continue
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()  
        process = launch.launch(node)
        process.stop()
        

if __name__ == '__main__':
    client_instance = action_client()
    try:
        client_instance.main()
    except rospy.ROSInterruptException:
        pass