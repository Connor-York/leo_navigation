#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
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
        self.status_dict = {v: k for k, v in GoalStatus.__dict__.items() if not k.startswith('_')}  #Converts GoalStatus values to their 
            
        # TODO: add csv logging stuff here
        
        # Experiment Parameters            
        self.role = rospy.get_param("~robot_role")
        self.num_robots = rospy.get_param("~num_robots")
        self.time_to_end = rospy.get_param("~time_to_end") * 60.0
            
        # Communication
        self.inbox = queue.Queue()
        self.server_pub = rospy.Publisher('/server_pub', String, queue_size=10) 
        rospy.Subscriber('/server_sub', String, self.comms_cb)

        # Patrol
        self.patrolling = Patroller(self.id, package_path, self.server_pub, self.num_robots, self.start_time)
        
        # Search
        self.searching = Searcher()
        
        # Move Base
        self.nav_client = self.nav_init()
        
        self.robot_state = "patrolling"
        self.goal_active = False
        
        rospy.on_shutdown(self.cleanup)
        self.main_loop()
    
    
    def main_loop(self):
        """
        Main robot loop
        """
        rate = rospy.Rate(25) # TODO set HZ here to be closer to the signal reading speed
        while not rospy.is_shutdown():
            elapsed_time = time.time() - self.start_time
            
            self.read_inbox()
            self.nav_check() #check current goal status and act accordingly
            
            if elapsed_time >= self.time_to_end:
                rospy.loginfo("Shutting down. Patrol Time Elapsed")
                rospy.signal_shutdown("Patrol Time Elapsed")
                
            # MEASURE SIGNAL
                
            if self.robot_state == "patrolling":
                if self.goal_active == False:
                    self.goal_active = True
                    goal = self.patrolling.set_nav_goal()
                    self.nav_client.send_goal(goal, done_cb=self.nav_cb)
                    rospy.loginfo("Sending goal")
                   
                # If no goal set, and at node? set goal, check if state is succeeded or failed and handle.
                # return goal via self.patrolling.sebs or cgg
                # send goal using self.set_nav_goal

                
            # elif self.robot_state == "searching":
            #     pass
        
        rate.sleep()
            
    
    def nav_init(self):
        """
        Initialise movebase client "nav_client" to send goals to
            Called in self.__init__
            Returns simpleactionclient object
        """
        nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = nav_client.wait_for_server(rospy.Duration(0.0))
        if not wait:
            rospy.logfatal("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        return nav_client
    
    def nav_check(self):
        """
        Check on current nav goal triggered each main loop
            Calls relevant next step decision (patroller arrived at node, searcher next step)
            state is 
        """
        state = self.nav_client.SimpleClientGoalState
        if state 
        rospy.loginfo(f"- GOAL - Finished with status: {self.status_dict.get(state, 'UNKNOWN')}")
        self.goal_active = False
        if state == GoalStatus.SUCCEEDED:
            if self.robot_state == 'patrolling':
                self.patrolling.arrived_at_node()
                rospy.loginfo(f"Goal active: {self.goal_active}")
        else:
            rospy.logerr(f"- GOAL - FAILURE, STATUS: {self.status_dict.get(state, 'UNKNOWN')}")
    
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
            
        
    def cleanup(self):
        rospy.loginfo("Shutting down: cleaning up resources...")
        
        # End any threads here
        
        if self.nav_client:
            self.nav_client.cancel_all_goals()

        rospy.loginfo("Cleanup complete.")
        
        
    

    
if __name__ == '__main__':
    Main()
    
    
    