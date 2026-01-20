#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import time
import threading
import queue
import json
import csv
import os
import numpy as np

# CLASSES
from robot_classes.patrol_class import Patroller
from robot_classes.search_class import Searcher


class Main:
    
    def __init__(self):
        rospy.init_node('Robot_Main')
        
        rpkg = rospkg.RosPack()
        self.package_path = rpkg.get_path("leo_navigation")
        
        # Logging
        self.start_time = time.time()
        self.id = rospy.get_param("~robot_id")
        trial_no = str(rospy.get_param("~trial_no"))
        trial_scenario = rospy.get_param("~trial_scenario")
        self.status_dict = {v: k for k, v in GoalStatus.__dict__.items() if not k.startswith('_')}  #Converts GoalStatus values to their tstring
        self.save_path = f"{self.package_path}/logs/Robot{self.id}_{trial_scenario}_{trial_no}"    
        
        self.signallog = []
        self.idlenesslog = []    
            
        # TODO: add csv logging stuff here
        
        # Experiment Parameters            
        self.role = rospy.get_param("~robot_role")
        self.num_robots = rospy.get_param("~num_robots")
        self.time_to_end = rospy.get_param("~time_to_end") * 60.0
        self.search_end_timer = rospy.get_param("~search_end_timer") * 60.0
        self.patrollers_measure = rospy.get_param("~patrollers_measure")
            
        # Communication
        self.inbox = queue.Queue()
        self.server_pub = rospy.Publisher('/server_pub', String, queue_size=10) 
        rospy.Subscriber('/server_sub', String, self.comms_cb)
        
        self.gbest_pub_point = rospy.Publisher(f'/gbest_point', PointStamped, queue_size=10)
        self.gbest_pub_text = rospy.Publisher(f'/gbest_text', Marker, queue_size=10)
        
        # Move Base
        self.nav_client = self.nav_init()
        
        # Robot State Information
        self.robot_state = "patrolling" # searching / patrolling
        self.patrol_flag = False
        self.search_flag = False
        
        rospy.on_shutdown(self.cleanup)
        
        # Wait for start message from server if needed
        self.comm_start = rospy.get_param("~comm_start")
        if self.comm_start == True:
            rospy.loginfo("================== Waiting for start message from server to begin... ==================")
            self.ok_start = False
        else:
            self.ok_start = True
        
        # Begin
        self.main_loop()
    
    
    def main_loop(self):
        """
        Main robot loop
        """
        rate = rospy.Rate(100) # signal reading spead is ~20Hz
        
        while self.ok_start == False and not rospy.is_shutdown():
            # Wait for start message from server (or just start immediately if comm_start == False)
            self.read_inbox(time.time())
            rate.sleep()

        rate = rospy.Rate(20) # signal reading spead is ~20Hz
        rospy.loginfo("=== Okay, let's go ===")
            
        self.start_time = time.time() 

        # Patrol
        self.patrolling = Patroller(self.id, self.package_path, self.server_pub, self.num_robots, self.start_time)
        
        # Search
        step_distance = rospy.get_param("~step_distance") # distance in metres to step each search step 
                            # (robot top speed is 0.4m/s so this is 1s of moving forward)
        self.searching = Searcher(step_distance, self.num_robots, self.id, self.server_pub, self.start_time)
        
        # Main Loop Proper
        while not rospy.is_shutdown():
            
            if self.patrol_flag or self.search_flag:
                rospy.logerr(f"Flag set at start of loop: Patrol: {self.patrol_flag} | Search: {self.search_flag} ")
                
            if self.inbox.qsize() > 10:
                rospy.logwarn(f"INBOX SIZE WARNING: Inbox size is {self.inbox.qsize()} messages")
            
            curr_time = time.time()
            elapsed_time = curr_time - self.start_time
            
            # Check for end of experiment
            if elapsed_time >= self.time_to_end:
                rospy.loginfo(f"Shutting down. Patrol Time Elapsed - T+{elapsed_time/60}s")
                rospy.signal_shutdown("Patrol Time Elapsed")
                break
            
            # read comms and signal, check if you found the source, update state accordingly
            self.read_inbox(curr_time)            
            self.read_signal(curr_time)
            if self.searching.source_found[0] == False:
                self.check_found(curr_time) 
            
            if self.patrol_flag or self.search_flag:    
                rospy.loginfo(f" Flags - Patrol: {self.patrol_flag} | Search: {self.search_flag} ")
            if self.patrol_flag and self.search_flag:
                rospy.logerr("Flags are both set and should not be simultaneously")
                

            self.searching.send_signal_message() 
            self.update_logs(curr_time, elapsed_time) 
            self.publish_gbest_visual() # publish for RVIZ visualisation (debugging)
            

            # Get next goal
            goal = None
            if self.robot_state == "patrolling":
                
                # Switch to searching if needed (flag set from read_signal or read_inbox)
                if self.search_flag:
                    rospy.loginfo("- STATE CHANGE - Patrolling -> Searching")
                    self.robot_state = "searching"
                    self.search_flag = False
                    # cancel goals, let other agents know you have no intention
                    self.nav_client.cancel_all_goals()
                    self.patrolling.current_goal = -1
                    self.patrolling.send_sebs_msg(None, None)
                    goal = self.searching.search_step()
                
                else:
                    goal_state = self.nav_client.get_state()
                    if goal_state == GoalStatus.SUCCEEDED:
                        goal = self.patrolling.arrived_at_node()
                    elif goal_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
                        rospy.logerr(f"- GOAL - FAILURE, STATUS: {self.status_dict.get(goal_state, 'UNKNOWN')} --")
                    elif goal_state == GoalStatus.LOST:
                        rospy.logwarn(f"- GOAL - None set, setting to current goal node (should only happen at start) ")
                        goal = self.patrolling.set_nav_goal()
            
            elif self.robot_state == "searching":
                
                # Switch to patrolling if needed (flag set from check_found or read_inbox)
                if self.patrol_flag:
                    rospy.loginfo("- STATE CHANGE - Searching -> Patrolling")
                    self.robot_state = "patrolling"
                    self.patrol_flag = False
                    self.nav_client.cancel_all_goals()
                    goal = self.patrolling.return_to_patrol(self.searching.pose_x, self.searching.pose_y)

                else:
                    goal_state = self.nav_client.get_state()
                    if goal_state == GoalStatus.SUCCEEDED:
                        goal = self.searching.search_step()
                    elif goal_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
                        rospy.logerr(f"- GOAL - FAILURE, STATUS: {self.status_dict.get(goal_state, 'UNKNOWN')} --")
                    elif goal_state == GoalStatus.LOST:
                        rospy.logwarn(f"- GOAL - None set ")
            
            if goal is not None:
                self.nav_client.send_goal(goal)
                        
            rate.sleep()
            
    
    def nav_init(self):
        """
        Initialise movebase client "nav_client" to send goals to
            Called in self.__init__
            Returns simpleactionclient object
            !! Something here is updating /amcl/initial_pose_x/y/a params ??
            Probably fine but just noting if it becomes an issue later.
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
    
    
    # def nav_check(self):
    #     """
    #     Check on current nav goal triggered each main loop
    #         Calls relevant next step decision (patroller arrived at node, searcher next step)
    #     """
    #     goal_state = self.nav_client.get_state()
    #     if goal_state in [3,4,5,8]: # terminal states
    #         rospy.loginfo(f"- GOAL - Finished with status: {self.status_dict.get(goal_state, 'UNKNOWN')} ")
    #         rospy.loginfo("=======================================================")
    #         self.goal_active = False
    #         if goal_state == GoalStatus.SUCCEEDED:
    #             if self.robot_state == 'patrolling':
    #                 self.patrolling.arrived_at_node()
                
    #         # elif state == GoalStatus.ABORTED: TODO add handling for aborted goal, 
    #         # likely to occur from multi-robot tests/ search behaviour -- Perhaps a delay and retry?, 
    #         # continue to next node without "going there" for patrol?
    #         else:
    #             rospy.logerr(f"- GOAL - FAILURE, STATUS: {self.status_dict.get(goal_state, 'UNKNOWN')} --")
    #             rospy.loginfo("=======================================================")
    
    
    def comms_cb(self, msg):
        """
        Callback to handle communications, queues them to agent inbox
        """
        data = json.loads(msg.data)
        self.inbox.put(data)
    
    
    def read_inbox(self, curr_time):
        """
        Empties inbox passing messages to relevant handlers, called every loop
        """
        while True:
            try:
                message = self.inbox.get_nowait()
            except queue.Empty:
                break
            
            rospy.loginfo(f"- COMS - Received message: {message}")
            
            # Handle server messages (start, signal_on) (prior to main loop, and in main loop)
            if message.get("source") == "server":
                if message.get("message") == "start" and self.comm_start == True:
                    self.ok_start = True
                elif message.get("message") == "signal_on":
                    self.searching.signal_start = True
                    rospy.loginfo("=== SIGNAL START ===")
                else:
                    rospy.logwarn(f"- COMS - Unknown server message: {message}")
                
            # Once start allowed, handle robot messages (main loop)
            if self.ok_start == True:
                if message.get("type") == "sebs":
                    self.patrolling.receive_sebs_message(message)
                    
                elif message.get("type") == "signal":
                    new_state = self.searching.receive_signal_message(message, self.robot_state, curr_time)
                    if new_state is not None and self.role == 'searcher':
                        if new_state == 'searching':
                            self.search_flag = True
                        elif new_state == 'patrolling':
                            self.patrol_flag = True

                
    def read_signal(self, curr_time):
        """
        Reads signal data from searching class, updates state if needed
        """
        new_state = self.searching.read_signal(curr_time, self.robot_state,True if (self.role == 'searcher' or self.patrollers_measure ) else False)
        if self.role == 'searcher':
            # State change from read_signal is only patrol -> search (otherwise new_state = None)
            if new_state is not None:
                self.search_flag = True
            

    def check_found(self, curr_time):
        """
        Checks if gbest_timer has ran out without a new gbest update, if so declares source found
        """
        if curr_time - self.searching.last_gbest_update > self.search_end_timer:
            rospy.loginfo(f"========================================================FOUND IT. GBEST IS: {self.searching.g_best[0]}")
            if self.searching.isbest:
                rospy.loginfo("Source found :)")
                self.searching.source_found = (True, self.searching.id)
                # Found it, return to patrol
                self.patrol_flag = True
            else:
               rospy.logwarn("Im not even best ? ")
            
    def update_logs(self, curr_time, elapsed_time):
        """
        Updates log data each main loop
        """
        if self.robot_state == 'patrolling':
            self.idlenesslog.append([curr_time, self.patrolling.current_goal, self.patrolling.node_idleness.copy()])
        self.signallog.append([elapsed_time, self.robot_state, self.searching.pose_x, self.searching.pose_y, self.searching.pose_yaw, self.searching.rssi_avg, self.searching.rssi_raw, self.searching.p_best[0], self.searching.p_best[1], self.searching.g_best[0], self.searching.g_best[1], self.searching.source_found])
        

    def log_data(self):
        if not os.path.exists(self.save_path):
            os.mkdir(self.save_path)
        
        with open(f"{self.save_path}/signallog.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['elapsed_time', 'robot_state', 'x', 'y', 'yaw', 'rssi_avg', 'rssi_raw', 'pbest_val', 'pbest_pos', 'gbest_val', 'gbest_pos', 'signal_found'])
            for row in self.signallog:
                writer.writerow(row)
        
        with open(f"{self.save_path}/idlenesslog.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['elapsed_time', 'current_goal', 'idleness'])
            
            for row in self.idlenesslog:
                current_time = row[0]
                
                # Convert to time since last visit using NumPy and round to 2 decimal places
                idleness_time_since = np.round(current_time - np.array(row[2]), 2)
                
                # Convert elapsed time
                elapsed_time = np.round(current_time - self.start_time, 2)
                
                # Write row (convert numpy array to list for CSV writing)
                writer.writerow([elapsed_time, row[1], idleness_time_since.tolist()])
                
        with open(f"{self.save_path}/robot_params.txt", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow("Robot Parameters: ")
            writer.writerow(f"ID: {self.id}")
            writer.writerow(f"Trial No: {rospy.get_param('~trial_no')}")
            writer.writerow(f"Scenario: {rospy.get_param('~trial_scenario')}")
            writer.writerow(f"Robot Role: {self.role}")
            writer.writerow(f"Search Method: {rospy.get_param('~search_method')}")
            writer.writerow(f"Number of Robots: {self.num_robots}")
            writer.writerow(f"Time to End (s): {self.time_to_end}")
            writer.writerow(f"Search End Timer (s): {self.search_end_timer}")
            writer.writerow(f"Patrollers Measure: {self.patrollers_measure}")
            
    def publish_gbest_visual(self):
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "map"
        point_msg.point.x = self.searching.g_best[1][0]
        point_msg.point.y = self.searching.g_best[1][1]
        point_msg.point.z = 0
        
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "sensor_values"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # Position text slightly above your point
        text_marker.pose.position.x = point_msg.point.x
        text_marker.pose.position.y = point_msg.point.y
        text_marker.pose.position.z = point_msg.point.z + 0.2  # Slightly above
        text_marker.pose.orientation.w = 1.0
        
        text_marker.text = f"{self.searching.g_best[0]}"
        
        text_marker.scale.z = 0.2
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        
        self.gbest_pub_point.publish(point_msg)
        self.gbest_pub_text.publish(text_marker)
        
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
    
    
    
