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
        self.id = rospy.get_param("~robot_id")
        trial_no = str(rospy.get_param("~trial_no"))
        trial_scenario = rospy.get_param("~trial_scenario")
        self.status_dict = {v: k for k, v in GoalStatus.__dict__.items() if not k.startswith('_')}  #Converts GoalStatus values to their tstring
        self.save_path = f"{self.package_path}/logs/Robot{self.id}_{trial_scenario}_{trial_no}"    
        
        self.signallog = []
            
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
            self.server_pub.publish(json.dumps({'source': self.id, 'type': 'ready'}))
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
            self.read_inbox()
            rate.sleep()

        curr_time = prev_message_time = self.start_time = time.time()
        rospy.loginfo(f"=== START! | T = {self.start_time} ===")
        message_rate = max(0.05, 1.0 / rospy.get_param("~message_rate")) # seconds between messages, min 0.05s (20Hz)
        rate = rospy.Rate(20) # signal reading spead is ~20Hz
        

        # Patrol
        self.patrolling = Patroller(self.id, self.package_path, self.server_pub, self.num_robots, curr_time)
        
        # Search
        step_distance = rospy.get_param("~step_distance") # distance in metres to step each search step 
                            # (robot top speed is 0.4m/s so this is 1s of moving forward)
        self.searching = Searcher(step_distance, self.num_robots, self.id, self.server_pub, curr_time)
        
        # Main Loop Proper
        while not rospy.is_shutdown():
            
            if self.patrol_flag or self.search_flag:
                rospy.logerr(f"Flag set at start of loop: Patrol: {self.patrol_flag} | Search: {self.search_flag} ")
                
            if self.inbox.qsize() > 5:
                rospy.logwarn(f"INBOX SIZE WARNING: Inbox size is {self.inbox.qsize()} messages")
            
            # Check for end of experiment
            if time.time() - self.start_time >= self.time_to_end:
                rospy.loginfo(f"Shutting down. Patrol Time Elapsed - {rospy.get_param('~time_to_end')} + {(time.time() - self.start_time)}s")
                rospy.signal_shutdown("Patrol Time Elapsed")
                break
            
            if (time.time() - self.start_time) % 60.0 < 0.1:
                rospy.loginfo(f"Patrol Time Elapsed: {(time.time() - self.start_time)/60.0:.2f} minutes")
            
            # read comms and signal, check if you found the source, update state accordingly
            self.read_inbox()            
            self.searching.publish_teammate_obstacles()
            self.read_signal()
            if self.searching.source_found[0] == False:
                self.check_found() 
            
            curr_time = time.time()
            if curr_time - prev_message_time >= message_rate:
                self.searching.send_signal_message(curr_time) 
                prev_message_time = curr_time
            
            if self.patrol_flag or self.search_flag:     # fat error catch 
                rospy.loginfo(f" Flags - Patrol: {self.patrol_flag} | Search: {self.search_flag} ")
                if self.patrol_flag and self.robot_state == "patrolling":
                    rospy.logerror("Patrol flag set while already patrolling")
                elif self.search_flag and self.robot_state == "searching":
                    rospy.logwarn("Search flag set while already searching")
                    
                if self.patrol_flag and self.search_flag:
                    rospy.logerr("Flags are both set and should not be simultaneously")
                

            self.update_logs() 
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
                        goal = self.patrolling.arrived_at_node(self.searching.pose_x, self.searching.pose_y)
                    elif goal_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
                        rospy.logerr(f"- GOAL - FAILURE, STATUS: {self.status_dict.get(goal_state, 'UNKNOWN')} -- TRYING AGAIN")
                        goal = self.patrolling.set_nav_goal(self.searching.pose_x, self.searching.pose_y) # try again :? 
                    elif goal_state == GoalStatus.LOST:
                        rospy.logwarn(f"- GOAL - None set, setting to current goal node (should only happen at start) ")
                        goal = self.patrolling.set_nav_goal(self.searching.pose_x, self.searching.pose_y)
            
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
                        self.searching.current_goal = goal  # Store for re-attempt if needed
                    elif goal_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
                        rospy.logerr(f"- GOAL - FAILURE, STATUS: {self.status_dict.get(goal_state, 'UNKNOWN')} -- TRYING AGAIN")
                        goal = self.searching.current_goal 
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
    
    
    def comms_cb(self, msg):
        """
        Callback to handle communications, queues them to agent inbox
        """
        data = json.loads(msg.data)
        #rospy.loginfo(f"Received message: t_sent {data.get('t_sent', 'N/A')} | raw: {data.get('raw_time', 'N/A')} | diff_cb {time.time() - data.get('raw_time', 0.0)}")
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
            
            #rospy.loginfo(f"- COMS - Received message: {message}")
            
            # Handle server messages (start, signal_on) (prior to main loop, and in main loop)
            if message.get("source") == "server":
                if message.get("message") == "start" and self.comm_start == True:
                    self.ok_start = True
                elif message.get("message") == "signal_on":
                    rospy.loginfo("=== SIGNAL START ===")
                    self.searching.signal_start = True
                else:
                    rospy.logwarn(f"- COMS - Unknown server message: {message}")
                
            # Once start allowed, handle robot messages (main loop)
            if self.ok_start == True:
                if message.get("type") == "sebs":
                    self.patrolling.receive_sebs_message(message)
                    
                elif message.get("type") == "signal":
                    new_state = self.searching.receive_signal_message(message, self.robot_state)
                    if new_state is not None and self.role == 'searcher':
                        if new_state == 'searching':
                            self.search_flag = True
                        elif new_state == 'patrolling':
                            self.patrol_flag = True

                
    def read_signal(self):
        """
        Reads signal data from searching class, updates state if needed
        """
        new_state = self.searching.read_signal(self.robot_state, True if (self.role == 'searcher' or self.patrollers_measure ) else False)
        if self.role == 'searcher':
            # State change from read_signal is only patrol -> search (otherwise new_state = None)
            if new_state is not None:
                self.search_flag = True
            

    def check_found(self):
        """
        Checks if gbest_timer has ran out without a new gbest update, if so declares source found
        """
        if self.searching.g_best[0] > -999 and self.searching.isbest:
            if (time.time() - self.searching.last_gbest_update > self.search_end_timer):
                rospy.loginfo(f"========================================================FOUND IT. GBEST IS: {self.searching.g_best[0]}")
                rospy.loginfo("Source found :)")
                self.searching.source_found = (True, self.searching.id)
                # Found it, return to patrol
                if self.robot_state == 'searching':
                    self.patrol_flag = True
            elif (time.time() - self.searching.last_gbest_update) % 30.0 < 0.1:
                rospy.loginfo(f"I have been GBest for {(time.time() - self.searching.last_gbest_update)/60.0:.2f} mins")
             
    def update_logs(self):
        """
        Updates log data each main loop
        """
        curr_time = time.time()
        self.patrolling.idlenesslog.append([curr_time, self.patrolling.current_goal, self.patrolling.node_idleness.copy(), 'reg'])
        self.signallog.append([curr_time, self.robot_state, self.searching.pose_x, self.searching.pose_y, self.searching.pose_yaw, self.searching.rssi_avg, self.searching.rssi_raw, self.searching.p_best[0], self.searching.p_best[1][0], self.searching.p_best[1][1], self.searching.g_best[0], self.searching.g_best[1][0], self.searching.g_best[1][1], self.searching.source_found[0]])

    def log_data(self):
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        
        with open(f"{self.save_path}/signallog.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'robot_state', 'x', 'y', 'yaw', 'rssi_avg', 'rssi_raw', 'pbest_val', 'pbest_x', 'pbest_y', 'gbest_val', 'gbest_x', 'gbest_y', 'signal_found'])
            for row in self.signallog:
                writer.writerow(row)
        
        with open(f"{self.save_path}/idlenesslog.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'current_goal'] + [f'node_{i}' for i in range(len(self.patrolling.node_list))] + ['source'])
            
            for row in self.patrolling.idlenesslog:
                current_time = row[0]
                idleness_time_since = current_time - np.array(row[2])
                # Write row (convert numpy array to list for CSV writing)
                writer.writerow([current_time, row[1]] + idleness_time_since.tolist() + [row[3]])
                
        with open(f"{self.save_path}/sebs_sent_log.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'position', 'intention'])
            for row in self.patrolling.sent_log:
                writer.writerow(row)
                
        with open(f"{self.save_path}/sebs_rec_log.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'source', 'time_msg', 'position', 'intention'])
            for row in self.patrolling.rec_log:
                writer.writerow(row)

        with open(f"{self.save_path}/robot_params.txt", mode='w', newline='') as file:
            file.write("Robot Parameters: \n")
            file.write(f"ID: {self.id}\n")
            file.write(f"Trial No: {rospy.get_param('~trial_no')}\n")
            file.write(f"Scenario: {rospy.get_param('~trial_scenario')}\n")
            file.write(f"Robot Role: {self.role}\n")
            file.write(f"Search Method: {rospy.get_param('~search_method')}\n")
            file.write(f"Number of Robots: {self.num_robots}\n")
            file.write(f"Time to End (s): {self.time_to_end}\n")
            file.write(f"Search End Timer (s): {self.search_end_timer}\n")
            file.write(f"Patrollers Measure: {self.patrollers_measure}\n")
            file.write(f"Start Time: {self.start_time}\n")
            
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
    
    
    
