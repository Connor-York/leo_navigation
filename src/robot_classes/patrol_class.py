#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import math
import pandas as pd
import json
import numpy as np
import time

class Patroller:
    
    def __init__(self, id, package_path, server_pub, num_robots, start_time):
        
        self.id = id
        self.start_time = start_time
        
        # Initialise Patrolling 
        patrol_route = package_path + "/waypoints/" + rospy.get_param("~patrol_route")
        self.waypoint_list = self.waypoint_gen(patrol_route)
        self.current_goal = rospy.get_param("~start_node")
        self.patrol_method = rospy.get_param("~patrol_method")
        self.intention_table = np.empty(num_robots, dtype=int)
        self.node_idleness = np.full(len(self.waypoint_list), self.start_time) # node_idleness array is the last time each node was visited 
        
        self.server_pub = server_pub # For sending communications

        
    def waypoint_gen(self, waypoint_csv):
        """
        Generate list of poses from waypoint csv
        """
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
    
    
    def send_sebs_msg(self, current_node, arrival_time):
        """
        Constructs and sends a sebs message, to be called when agent arrives at a waypoint (goal node)
        """
        if current_node == self.current_goal:
            rospy.logerr(f"Current Node {current_node} is the same as goal {self.current_goal} when sending sebs message.")
        Message = { 
            'source':self.id,
            'type':"sebs",
            'position':current_node, 
            'intention':self.current_goal,
            'time':arrival_time
        }
        self.server_pub.publish(json.dumps(Message))
        rospy.loginfo("Sebs message sent!")
        
    def receive_sebs_msg(self, message):
        """
        Handles a received sebs message, updating intention table and believed node idleness
        """
        rospy.loginfo(f"Received SEBS message from {message["message"]}")
        self.intention_table[message["source"]] = message["intention"]
        self.node_idleness[message["position"]] = message["time"]
            
    def arrived_at_node(self):
        arrival_time = time.time()
        current_node = self.current_goal
        
        self.node_idleness[current_node] = arrival_time
        
    def sebs(self):
        pass
    
    def cgg(self):
        pass
        
        
        
    
    
        
    