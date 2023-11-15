#!/usr/bin/env python3

import rospy
import math
import pandas as pd
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
import time
import random
from ar_track_alvar_msgs.msg import AlvarMarkers
import datetime
import csv

class Patroller():

    def __init__(self):

        rospy.init_node('Robot_Main')  # initialize node

        #subscriber to listen to the ar_track_alvar node purely for checking position of tags
        #which are declared as other robots
        robot_locator = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.robot_locator_cb)
        self.robot_check_tick = 0 #If robot sees another robot, and skips to the next waypoint,
        #this check_tick makes sure it reaches that waypoint first before skipping another? 
        #may be flawed and may want to replace it with a time check ? 

        # Getting timing and information for logging ------------------------------
        self.start_time = time.time()
        current_time_save = datetime.datetime.now()
        current_time_save = current_time_save.strftime("%H:%M:%S")
        self.time_to_end = rospy.get_param("~time_to_end") * 60.0
        # ^ Time after which to end patrolling (if decided) received in minutes, then into seconds

        # gets csv file path
        rp = rospkg.RosPack()
        package_path = rp.get_path('leo_navigation')
        route = rospy.get_param('~route')
        CSV_path = (package_path + "/waypoints/" + route)

        name = rospy.get_param("~robot_name")
        trial_no = rospy.get_param("~trial_no")
        trial_no = str(trial_no)
        scenario = rospy.get_param("~trial_scenario")
        time_csv_name = (name + "_" + scenario + "_" + trial_no)
       
        self.time_csv_path = (package_path + "/logs/TIMES_" + time_csv_name + "_" + current_time_save + ".csv")
        self.reward_csv_path = (package_path + "/logs/REWARDS_" + time_csv_name + "_" + current_time_save + ".csv")
        self.skip_csv_path = (package_path + "/logs/SKIPPED_" + time_csv_name + "_" + current_time_save + ".csv")

        # preprocessing --------------------------------------------------
        # converts waypoints text file into a list of points to follow
        df = pd.read_csv(CSV_path, sep=',', header=None)
        self.theta = list(df.loc[:, 3].values)
        wayp = df.loc[:, 0:2]
        self.waypoints = []
        wayp = wayp.values.tolist()
        for sublist in wayp:
            for item in sublist:
                self.waypoints.append(item)

        points_seq = self.waypoints  # heading angle for each waypoint
        yaweulerangles_seq = self.theta  # coordinates for each waypoint

        # Convert waypoint & heading values into a list of robot poses (quaternions?) -----------
        quat_seq = list()
        # List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = rospy.get_param('~start_node')
        for yawangle in yaweulerangles_seq:
            # Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(
                *(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            # Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
            n += 1

        # Create action client -------------------------------------------
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        # Initiate status subscriber
        self.status_subscriber = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.status_cb
        )

        # Create a Twist publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        self.patrol_count = 0
        self.tick = 0

        self.state_patrolling()

    def status_cb(self, msg): #
        status = self.client.get_state()
        #rospy.loginfo(status)
        if status == 3:
            self.robot_check_tick = 0
            rospy.loginfo(" status tree ")
            self.client.stop_tracking_goal()
            rospy.loginfo("GOING TO WAYPOINT STATE")
            self.state_at_waypoint()

    def robot_locator_cb(self,msg): #callback triggered by ar_tags purely for checking other rob pos.
        other_robot_id = rospy.get_param("~other_robot_id")
        avoid_dist = rospy.get_param("~avoid_dist")
        for marker in msg.markers:
            if marker.id == other_robot_id:
                x = marker.pose.pose.position.x
                y = marker.pose.pose.position.y
                dist = math.sqrt(x**2 + y**2)
                print("Robot Spotted dist: " + str(dist))
                if dist <= avoid_dist and self.robot_check_tick == 0:
                    print("Robot spotted within avoid distance, moving to next waypoint")
                    time_skip_waypoint = time.time() - self.start_time
                    data = ["waypoint_skipped: "  + str(self.goal_cnt+1), time_skip_waypoint]
                    self.save_to_csv(self.skip_csv_path,data)
                    self.robot_check_tick = 1
                    self.continue_patrol()

    def state_patrolling(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose " +
                      str(self.goal_cnt+1)+" to Action Server")
        #rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal)
        rospy.loginfo("==========* GOAL SENT *==========")
        

    def state_at_waypoint(self):
        rospy.loginfo("At waypoint")

        #Save time at waypoint
        time_at_waypoint = time.time() - self.start_time 
        data = ["waypoint_" + str(self.goal_cnt+1),time_at_waypoint]
        self.save_to_csv(self.time_csv_path,data)

        ts = Tag_scan(self.start_time,self.reward_csv_path) #calls tag scan, does the thing, continues
        ts.tag_scan()
        rospy.loginfo("after tag scan") 
        if(ts.complete_scan == 1):
            rospy.loginfo("continuing patrol")
            self.continue_patrol()

    
    #checks robot patrol state, determines whether to continue looping or whether to return home
    # and stop. Can definitely be cleaned up but im not gonna do that now.
    def continue_patrol(self):
        current_time = time.time() - self.start_time
        if current_time >= self.time_to_end: # timed run completed. 
            rospy.loginfo("FIN")
            rospy.loginfo(reward_count)
            rospy.loginfo(reward_ID_seen)
            rospy.loginfo(no_reward_ID_seen)
            rospy.signal_shutdown("FIN")
            exit()

        self.goal_cnt +=1 #Increment goal count
        rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")

        if self.goal_cnt == len(self.pose_seq): #Loop waypoint list
                self.goal_cnt = 0

        if self.goal_cnt != rospy.get_param("~start_node"): #Check if returned home
            rospy.loginfo("Moving onto next goal...")
            self.state_patrolling()

        else:   #if completed a full loop
            rospy.loginfo("Final goal pose reached!")
            self.patrol_count += 1
            # if self.patrol_count == rospy.get_param("~patrols"): # if done all the patrols return home, set tick to 1
            #     # goal = MoveBaseGoal()
            #     # goal.target_pose.header.frame_id = "map"
            #     # goal.target_pose.header.stamp = rospy.Time.now()
            #     # goal.target_pose.pose = self.pose_seq[rospy.get_param("~start_node")]
            #     # self.client.send_goal(goal)
            #     self.goal_cnt = rospy.get_param("~start_node")
            #     self.tick = 1
            #     rospy.loginfo("==========* Returning Home *==========")
            #     self.state_patrolling()
            #else:
            rospy.loginfo("Repeating patrol ...")
            self.goal_cnt = rospy.get_param("~start_node")
            self.client.cancel_goal()
            self.state_patrolling()


    def spin_robot(self):
        # Spin robot on the spot
        t_end = rospy.Time.now() + rospy.Duration(8) #about 360deg
        rospy.loginfo("Spinning...")
        while rospy.Time.now() < t_end:
            vel_msg = Twist()
            vel_msg.angular.z = 7.0
            self.vel_pub.publish(vel_msg)
            #rospy.loginfo("midspin")

        rospy.loginfo("Spin Done")
        # Stop the robot
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)

    def save_to_csv(self,csv_path,data):
        with open(csv_path, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(data)

class Tag_scan(): #==============================================================================

    def __init__(self,start_time,reward_csv_path):
        self.ID_list = []
        self.buffer = []
        self.rewards = rospy.get_param("~rewards") #list of tags which give reward
        rewards_2 = rospy.get_param("~rewards_2") #second list to switch to after X time
        self.LI = rospy.get_param("~latent_inhibition") #0 = always rescan 1 = never rescan (floating val)

        self.num_tags = rospy.get_param("~num_tags") #number of tags to read at each waypoint

        self.start_time = start_time
        self.reward_csv_path = reward_csv_path

        current_time = time.time() - self.start_time

        time_switch = rospy.get_param("~time_switch") * 60 #Time after which to switch rewarding tags
        
        dynamic_env = rospy.get_param("~dynamic_env") #whether to switch tag rewards

        if current_time >= time_switch and dynamic_env == True:
            print("Environment switching ==================================================================")
            # print("Old rewards: ")
            # print(self.rewards)
            self.rewards = rewards_2
            # print("New rewards: ")
            # print(self.rewards) 

        self.ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.check_ID_callback)
        
        self.complete_scan = 0
        self.callback_tick = 1
        rospy.loginfo('finished init func')

    def tag_scan(self):

        #rospy.loginfo("TAG SCAN") 

        if self.callback_tick == 0:
            scanned_prev = reward_ID_seen + no_reward_ID_seen
            print(scanned_prev)
            print(reward_ID_seen)
            print(no_reward_ID_seen)
            rospy.loginfo(self.ID_list)
            for ID in self.ID_list: 
                # this logic is definitely flawed, test and fix to not
                # put tags in both? Or to handle the switching case.
                if ID not in scanned_prev: # if new, scan for the first time
                    rospy.loginfo("Scanning new ID - " + str(ID))
                    self.scan(ID)
                elif ID in reward_ID_seen: # if known reward, scan
                    rospy.loginfo("Scanning known reward ID - " + str(ID))
                    self.scan(ID)
                elif ID in no_reward_ID_seen: # only re-scan known no reward if chance 
                    chance = 1 - self.LI # High LI is low chance, LOW LI is high chance :) 
                    r = random.random() #float in range 0-1
                    print("CHANCE")
                    print(chance)
                    print(r)
                    if r <= chance: 
                        rospy.loginfo("Rescanning known no reward ID - " + str(ID))
                        self.scan(ID) #rescan :) 
                    else:
                        rospy.loginfo("Ignoring known no reward ID - " + str(ID))
            rospy.loginfo("scan forloop complete, complete_scan = 1")
            self.complete_scan = 1
            rospy.loginfo("Reward Count: " + str(reward_count))
        #rospy.loginfo("After for loop")
        
        
        rospy.sleep(1) #If i remove this it leaves tag scan too early I think.
        if self.complete_scan == 0:
            self.tag_scan()
        self.ar_subscriber.unregister() 
        rospy.loginfo("unsubscribed")

    def scan(self,ID): # GLOBALS EDITED HERE
        global reward_ID_seen
        global no_reward_ID_seen
        global reward_count
        self.scan_delay() # delay to simulate doing something lmao
        if ID in self.rewards:
            if ID not in reward_ID_seen:
                reward_ID_seen.append(ID)
            if ID in no_reward_ID_seen:
                no_reward_ID_seen.remove(ID)
            rospy.loginfo("Reward got!")
            reward_count += 1
            current_time = time.time() - self.start_time
            data = [reward_count,current_time]
            self.save_to_csv(self.reward_csv_path,data)
            
        else:
            if ID not in no_reward_ID_seen:
                no_reward_ID_seen.append(ID)
            if ID in reward_ID_seen:
                reward_ID_seen.remove(ID)
            rospy.loginfo("No Reward :(")


    def scan_delay(self):
        t = rospy.get_param("~scan_delay")
        for i in range(t):
            print("Scanning " + str(i+1) + "/" + str(t) + "...")
            rospy.sleep(1)
        rospy.loginfo("Scan complete")

    def save_to_csv(self,csv_path,data):
        with open(csv_path, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(data)

    def dupe_check(self, iterable,check):
        for x in iterable:
            if x == check:
                return True

    def buffer_check(self, check): 
        self.buffer.append(check)
        if len(self.buffer) == 11:
            self.buffer.pop(0)
        bick = 0 #buffer tick
        print("BUFFER CHECK: ")
        print(self.buffer)
        for x in self.buffer: 
            if x == check:
                bick += 1
                #print(bick)
                if bick >= 3: # if 3 of the same tag in buffer
                    return True

    def check_ID_callback(self, msg): #callback triggered by ar_tag subscriber
        if self.callback_tick == 1:
            for marker in msg.markers:
                #filter out fake IDs (>17), any IDs already in the list, and only accept those that have been seen thrice
                if self.buffer_check(marker.id):
                    if marker.id < 18:
                        if self.dupe_check(self.ID_list, marker.id) == None:
                            rospy.loginfo("CALLBACK ID")
                            #current_time = time.time()
                            #elapsed_time = current_time - start_time
                            #Time_list.append(elapsed_time)
                            self.ID_list.append(marker.id)
                            if len(self.ID_list) == self.num_tags:
                                self.callback_tick = 0



if __name__ == '__main__':
    try: 
        reward_ID_seen = []
        no_reward_ID_seen = []
        reward_count = 0

        Patroller()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")