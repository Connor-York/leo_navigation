#!/usr/bin/env python3

# ROS IMPORTS 
import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist

# GENERAL IMPORTS
import pandas as pd
import time
import math

# CLASSES
import patrol_class
import search_class
    

class main():
    
    def __init__(self):
        rospy.init_node('Robot_Main')
        rpkg = rospkg.RosPack()
        package_path = rpkg.get_path("leo_navigation")
        
        # Logging
        self.start_time = time.time()
        self.robot_name = str(rospy.get_param("~robot_name"))
        trial_no = str(rospy.get_param("~trial_no"))
        trial_scenario = rospy.get_param("~trial_scenario")
            
        # TODO: add csv logging stuff here
        
        # End Condition
        self.end_condition = rospy.get_param("~end_condition")
        if self.end_condition == "laps":
            self.patrol_laps = rospy.get_param("~patrol_laps")
        elif self.end_condition == "time":
            self.time_to_end = rospy.get_param("~time_to_end")
        else:
            rospy.logerr(f"end_condition '{self.end_condition}' not recognised.")
            
        # Communication
        

        # Patrol
        patrol_route = package_path + "/waypoints/" + rospy.get_param("~patrol_route")
        self.waypoint_list = self.waypoint_gen(patrol_route)
        self.current_goal = rospy.get_param("~start_node")
        self.current_lap = 0
        
        # Move Base
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.nav_client.wait_for_server(rospy.Duration(0.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        
        

    def waypoint_gen(self, waypoint_csv):
        # preprocessing --------------------------------------------------
        # converts waypoints text file into a list of points to follow
        df = pd.read_csv(waypoint_csv, sep=',', header=None)
        theta = list(df.loc[:, 3].values)
        wayp = df.loc[:, 0:2]
        waypoints = []
        wayp = wayp.values.tolist()
        for sublist in wayp:
            for item in sublist:
                waypoints.append(item)

        points_seq = waypoints  # coordinates for each waypoint
        yaweulerangles_seq = theta  # heading angle for each waypoint

        # Convert waypoint & heading values into a list of robot poses (quaternions?) -----------
        quat_seq = list()
        # List of goal poses:
        pose_seq = list()
        for yawangle in yaweulerangles_seq:
            # Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(
                *(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            # Exploit n variable to cycle in quat_seq
            pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
            n += 1
            
        return pose_seq
        
    

    
if __name__ == '__main__':
    main()
    
    
    