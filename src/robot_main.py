#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import threading
import queue
import json

# CLASSES
from robot_classes.patrol_class import Patroller
from robot_classes.search_class import Searcher
    

class Main:
    
    def __init__(self):
        rospy.init_node('Robot_Main')
        rospy.on_shutdown(self.cleanup)
        
        rpkg = rospkg.RosPack()
        package_path = rpkg.get_path("leo_navigation")
        
        # Logging
        self.start_time = time.time()
        self.id = rospy.get_param("~robot_id")
        trial_no = str(rospy.get_param("~trial_no"))
        trial_scenario = rospy.get_param("~trial_scenario")
            
        # TODO: add csv logging stuff here
        
        # Experiment Parameters            
        self.role = rospy.get_param("~robot_role")
        self.num_robots = rospy.get_param("~num_agents")
        self.time_to_end = rospy.get_param("~time_to_end") * 60.0
            
        # Communication
        self.inbox = queue.Queue()
        self.server_pub = rospy.Publisher('/server_pub', String, queue_size=10) 
        rospy.Subscriber('/server_sub', String, self.comms_cb)

        # Patrol
        self.patrolling = Patroller(self.id, package_path, self.server_pub, self.start_time)
        
        # Search
        self.searching = Searcher()
        
        # Move Base
        self.nav_client = self.nav_init()
        
        self.robot_state = "patrolling"
        self.goal_active = False
        self.main_loop()
    
    
    def main_loop(self):
        """
        Main robot loop
        """
        rate = rospy.Rate(25) # TODO set HZ here to be closer to the signal reading speed
        while not rospy.is_shutdown():
        
            elapsed_time = time.time() - self.start_time()
            
            self.read_inbox()
            
            if elapsed_time >= self.time_to_end:
                rospy.loginfo("Shutting down. Patrol Time Elapsed")
                rospy.signal_shutdown("Patrol Time Elapsed")
                
            # MEASURE SIGNAL
                
            if self.robot_state == "patrolling":
                # If no goal set, and at node? set goal, check if state is succeeded or failed and handle.
                # return goal via self.patrolling.sebs or cgg
                # send goal using self.set_nav_goal
                pass
                
            elif self.robot_state == "searching":
                pass
        
        rate.sleep()
            
    
    def cleanup(self):
        rospy.loginfo("Shutting down: cleaning up resources...")
        
        # End any threads here
        
        if self.nav_client:
            self.nav_client.cancel_all_goals()

        rospy.loginfo("Cleanup complete.")
    
    def nav_init(self):
        """
        Initialise movebase client "nav_client" to send goals to
        """
        nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = nav_client.wait_for_server(rospy.Duration(0.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        return nav_client
    
    def set_nav_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.current_goal]
        rospy.loginfo("Sending goal pose " + str(self.current_goal) + " to Action Server")
        self.nav_client.send_goal(goal)
        rospy.loginfo("==========* GOAL SENT *==========")
    
    def comms_cb(self, msg):
        """
        Callback to handle communications, queues them to agent inbox
        """
        data = json.loads(msg.data)
        self.inbox.put(data)
    
    def read_inbox(self):
        """
        Empties inbox passing messages to relevant handlers, called every loop
        """
        while True:
            try:
                message = self.inbox.get_nowait()
            except queue.Empty:
                break
            
            if message.get("type") == "sebs":
                self.patrolling.receive_sebs_message(message)
            
    
            
            
        
        
    

    
if __name__ == '__main__':
    Main()
    
    
    