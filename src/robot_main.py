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
        rpkg = rospkg.RosPack()
        package_path = rpkg.get_path("leo_navigation")
        
        # Logging
        self.start_time = time.time()
        self.id = rospy.get_param("~robot_id")
        trial_no = str(rospy.get_param("~trial_no"))
        trial_scenario = rospy.get_param("~trial_scenario")
            
        # TODO: add csv logging stuff here
        
        # Experiment Parameters
        self.end_condition = rospy.get_param("~end_condition")
        if self.end_condition == "laps":
            self.patrol_laps = rospy.get_param("~patrol_laps")
        elif self.end_condition == "time":
            self.time_to_end = rospy.get_param("~time_to_end")
        else:
            rospy.logerr(f"end_condition '{self.end_condition}' not recognised.")
            
        self.role = rospy.get_param("~robot_role")
        self.num_robots = rospy.get_param("~num_agents")
            
        # Communication
        self.server_pub = rospy.Publisher('/server_pub', String, queue_size=10) 
        self.inbox = queue.Queue()
        rospy.Subscriber('/server_sub', String, self.comms_cb)

        # Patrol
        self.patrolling = Patroller(self.id, package_path, self.server_pub)
        
        # Search
        self.searching = Searcher()
        
        # Move Base
        self.nav_client = self.nav_init()
    
    
    def nav_init(self):
        nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = nav_client.wait_for_server(rospy.Duration(0.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_cb)
        
        return nav_client
    
    def status_cb(self):
        pass
    
    def comms_cb(self, msg):
        data = json.loads(msg.data)
        self.inbox.put(data)
    
    def read_inbox(self):
        while True:
            try:
                message = self.inbox.get_nowait()
            except queue.Empty:
                break
            
            if message['type'] == "sebs":
                
        
    
            
            
        
        
    

    
if __name__ == '__main__':
    
    try:
        Main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("~fin~")
    
    
    