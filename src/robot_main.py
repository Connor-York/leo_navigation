#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
import time
import threading
import queue
import json
import csv
import os

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
        self.save_path = f"{package_path}/logs/Robot{self.id}_{trial_scenario}_{trial_no}"    
        
        self.signallog = []
        self.idlenesslog = []    
            
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
        step_distance = rospy.get_param("~step_distance") # distance in metres to step each search step 
                            # (robot top speed is 0.4m/s so this is 1s of moving forward)
        self.searching = Searcher(step_distance, self.num_robots, self.id, self.server_pub)
        
        # Move Base
        self.nav_client = self.nav_init()
        
        
        
        self.robot_state = "patrolling" # searching / patrolling
        self.goal_active = False
        rospy.on_shutdown(self.cleanup)
        
        self.main_loop()
    
    
    def main_loop(self):
        """
        Main robot loop
        """
        rate = rospy.Rate(20) # signal reading spead is ~20Hz
        previous_log_time = time.time() - self.start_time
        
        while not rospy.is_shutdown():
            elapsed_time = time.time() - self.start_time
            
            
            self.read_inbox()
            
            self.searching.read_signal()


            self.nav_check() #check current goal status and act accordingly
            
            
            
            #log data
            if elapsed_time - previous_log_time >= 1:
                self.searching.send_signal_message()
                log_time = time.time()
                
                self.signallog.append([elapsed_time, self.robot_state, self.searching.pose_x, self.searching.pose_y, self.searching.pose_yaw, self.searching.rssi_avg, self.searching.rssi_raw])
                if self.robot_state == 'patrolling':
                    self.idlenesslog.append([log_time, self.patrolling.current_goal, self.patrolling.node_idleness])
                previous_log_time = elapsed_time
                
            
            if elapsed_time >= self.time_to_end:
                rospy.loginfo("Shutting down. Patrol Time Elapsed")
                rospy.signal_shutdown("Patrol Time Elapsed")
                
                
            if self.robot_state == "patrolling":
                if self.goal_active == False:
                    self.goal_active = True
                    goal = self.patrolling.set_nav_goal()
                    self.nav_client.send_goal(goal)
            
            elif self.robot_state == "searching":
                if self.goal_active == False:
                    self.goal_active = True
                    if self.searching.search_method == 'PSO':
                        goal = self.searching.PSO_step()
                    elif self.searching.search_method == 'ECOLI':
                        goal = self.searching.ECOLI_step()
                    elif self.searching.search_method == 'HYBRID':
                        goal = self.searching.HYBRID_step()
                        
                    self.nav_client.send_goal(goal)
        
            rate.sleep()
            
    
    def nav_init(self):
        """
        Initialise movebase client "nav_client" to send goals to
            Called in self.__init__
            Returns simpleactionclient object
        """
        nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("- init - Waiting for move_base action server...")
        wait = nav_client.wait_for_server(rospy.Duration(0.0))
        if not wait:
            rospy.logfatal("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("- init - Connected to move base server")
        return nav_client
    
    
    def nav_check(self):
        """
        Check on current nav goal triggered each main loop
            Calls relevant next step decision (patroller arrived at node, searcher next step)
        """
        goal_state = self.nav_client.get_state()
        if goal_state in [3,4,5,8]: # terminal states
            rospy.loginfo(f"- GOAL - Finished with status: {self.status_dict.get(goal_state, 'UNKNOWN')} ")
            rospy.loginfo("=======================================================")
            self.goal_active = False
            if goal_state == GoalStatus.SUCCEEDED:
                if self.robot_state == 'patrolling':
                    self.patrolling.arrived_at_node()
                    
                    if self.role == 'searcher' and self.searching.rssi_avg > -999:
                        rospy.loginfo(f" STATE - CHANGE FROM PATROLLING TO SEARCHING")
                        self.robot_state = 'searching'
            # elif state == GoalStatus.ABORTED: TODO add handling for aborted goal, 
            # likely to occur from multi-robot tests/ search behaviour -- Perhaps a delay and retry?, 
            # continue to next node without "going there" for patrol?
            else:
                rospy.logerr(f"- GOAL - FAILURE, STATUS: {self.status_dict.get(goal_state, 'UNKNOWN')} --")
                rospy.loginfo("=======================================================")
    
    
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
            
            if message.get("type") == "sebs" and self.robot_state == 'patrolling':
                self.patrolling.receive_sebs_message(message)
                
            if message.get("type") == "signal":
                new_state = self.searching.receive_signal_message(message, self.robot_state)
                if new_state is not None and self.role == 'searcher':
                    rospy.loginfo(f"- COMS - CHANGING STATE FROM {self.agent_state} to {new_state}")
                    self.agent_state = new_state
            

    def log_data(self):
        if not os.path.exists(self.save_path):
            os.mkdir(self.save_path)
        
        with open(f"{self.save_path}/signallog.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['elapsed_time', 'robot_state', 'x', 'y', 'yaw', 'rssi_avg', 'rssi_raw'])
            for row in self.signallog:
                writer.writerow(row)
        
        with open(f"{self.save_path}/idlenesslog.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['elapsed_time', 'current_goal', 'idleness'])
            for row in self.idlenesslog:
                # convert from last visit timestamp to time since
                #rospy.loginfo(f"Row: {row}")
                for item in row[2]:
                    
                    #rospy.loginfo(f"Item: {item}")
                    item = row[0] - item
                    #rospy.loginfo(f"Item a: {item}")
                row[0] = row[0] - self.start_time
                writer.writerow(row)
                
        with open(f"{self.save_path}/robot_params.txt", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow("Robot Parameters: ")
            writer.writerow(f"ID: {self.id}")
            writer.writerow(f"Trial No: {rospy.get_param('~trial_no')}")
            writer.writerow(f"Scenario: {rospy.get_param('~trial_scenario')}")
            writer.writerow(f"Robot Role: {self.role}")
            writer.writerow(f"Search Method: {self.searching.search_method}")
            
            
        
    def cleanup(self):
        rospy.loginfo("Shutting down: cleaning up resources...")
        
        # End any threads here
        rospy.loginfo("Saving log data...")
        self.log_data()
        
        if self.nav_client:
            self.nav_client.cancel_all_goals()

        rospy.loginfo("Cleanup complete.")
        
        
    

    
if __name__ == '__main__':
    Main()
    
    
    