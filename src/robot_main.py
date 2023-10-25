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

class Patroller():

    def __init__(self):

        rospy.init_node('patroller')  # initialize node

        # preprocessing --------------------------------------------------

        # gets csv file path
        rp = rospkg.RosPack()
        package_path = rp.get_path('leo_navigation')
        route = rospy.get_ptimearam('~route')
        CSV_path = (package_path + "/waypoints/" + route)

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

    def status_cb(self, msg):
        status = self.client.get_state()
        #rospy.loginfo(status)
        if status == 3:
            self.state_at_waypoint():

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
        #do something? 
        Tag_scan() #calls tag scan, class, cause uh, idk really.

        self.continue_patrol()

    def continue_patrol(self):
    #checks robot patrol state, determines whether to continue looping or whether to return home
    # and stop. Can definitely be cleaned up but im not gonna do that now.
        if self.tick == 1: # tick is one, returned home, done.
            rospy.loginfo("FIN")
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
            if self.patrol_count == rospy.get_param("~patrols"): # if done all the patrols return home, set tick to 1
                # goal = MoveBaseGoal()
                # goal.target_pose.header.frame_id = "map"
                # goal.target_pose.header.stamp = rospy.Time.now()
                # goal.target_pose.pose = self.pose_seq[rospy.get_param("~start_node")]
                # self.client.send_goal(goal)
                self.goal_cnt = rospy.get_param("~start_node")
                self.tick = 1
                rospy.loginfo("==========* Returning Home *==========")
                self.state_patrolling()
            else:
                rospy.loginfo("Repeating patrol ...")
                self.goal_cnt = rospy.get_param("~start_node")
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

class Tag_scan(): #==============================================================================

    def __init__(self):
        self.ID_list = []
        self.buffer = []
        rewards = [1,2,3,4]

        self.num_tags = 4

        self.ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, check_ID_callback)

        self.tag_scan():

    def tag_scan(self):
        while(len(self.ID_list)!=self.num_tags){ # wait 
        }
        self.ar_subscriber.unregister() # stop scanning tags after 4 unique scanned

        scanned_prev = reward_ID_seen + no_reward_ID_seen

        for ID in self.ID_list: 
            # this logic is definitely flawed, test and fix to not
            #put tags in both? Or to handle the switching case.
            if ID not in scanned_prev: # if new, scan for the first time
                self.scan(ID)
            elif ID in reward_ID_seen: # if known reward, scan
                self.scan(ID)
                continue #hm
            elif ID in no_reward_ID_seen: # only re-scan known no reward if chance 
                chance = 1 - self.LI # High LI is low chance, LOW LI is high chance :) 
                r = random.random() #float in range 0-1
                if r <= chance: 
                    self.scan(ID) #rescan :) 



    def scan(self,ID,tick):
        if ID in rewards:
            reward_ID_seen.append(ID)
            reward_count += 1
        else:
            no_reward_ID_seen.append(ID)
        self.scan_delay()

    def scan_delay(self):
        for i in range(10):
            print("Scanning... ", i+1)
            time.sleep(1)
        print("Scan complete")


    def dupe_check(self, iterable,check):
        for x in iterable:
        if x == check:
            return True

    def buffer_check(self, check): 
        self.buffer.append(check)
        if len(self.buffer) == 11:
            self.buffer.pop(0)
        bick = 0 #buffer tick
        for x in buffer: 
            if x == check:
                bick += 1
                if bick == 3: # if 3 of the same tag in buffer
                    return True

    def check_ID_callback(self, msg): #main callback, does everything
        for marker in msg.markers:

            # These two just print the ID and Pose to the cmd line
            #rospy.loginfo(marker.id)
            #rospy.loginfo(marker.pose.pose)
            #print(marker.id)
            #print(buffer)
            #print(ID_list)
        
            #filter out fake IDs (>17), any IDs already in the list, and only accept those that have been seen thrice
            if buffer_check(marker.id):
                if marker.id < 18:
                    if dupe_check(ID_list, marker.id) == None:
                        #print("ACCEPTED")
                        #current_time = time.time()
                        #elapsed_time = current_time - start_time
                        #Time_list.append(elapsed_time)
                        self.ID_list.append(marker.id)

    

if __name__ == '__main__':
    try:
        global reward_ID_seen = []
        global no_reward_ID_seen = []
        global reward_count = 0

        Patroller():
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
