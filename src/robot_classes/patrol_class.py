#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseGoal
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
        adjacency = package_path + "/waypoints/" + rospy.get_param("~patrol_adjacency")
            #Generate waypoint list and adjacency matrix
        self.node_list = self.waypoint_gen(patrol_route)
        self.node_neighbours = self.neighbours_gen(adjacency)
        
        self.start_node = rospy.get_param("~start_node")
        self.current_goal = self.start_node
        self.patrol_method = rospy.get_param("~patrol_method")
        if self.patrol_method not in ["CGG","SEBS"]:
            rospy.logfatal(f"Patrol method: {self.patrol_method} not recognised.")
            rospy.signal_shutdown(f"Invalid patrol method.")
            
        self.intention_table = np.empty(num_robots, dtype=int)
        self.node_idleness = np.full(len(self.node_list), self.start_time, dtype=np.float64) # node_idleness array is the last time each node was visited 
        
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
        yaweulerangles_seq = theta  # heading angle for each waypoint in radians

        # Converting headings to quarternians from radians and forming them into a list
        quat_seq = list()
        # List of goal poses:
        pose_seq = list()
        for yawangle in yaweulerangles_seq:
            # Taking the yaw, converting it to quarternian
            quat_seq.append(Quaternion(
                *(quaternion_from_euler(0, 0, yawangle, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            # Exploit n variable to cycle in quat_seq
            pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
            n += 1
            
        return pose_seq
    
    def set_nav_goal(self):
        """
        Set Nav goal to current_goal
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.node_list[self.current_goal]
        rospy.loginfo(f"- GOAL - Navigating to node: {self.current_goal}")
        return goal
    
    def send_sebs_msg(self, current_node, arrival_time):
        """
        Constructs and sends a sebs message, to be called when agent arrives at a goal node
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
        rospy.loginfo(f"- COMMS - Sebs message sent: ")
        rospy.loginfo(f"Position: {Message['position']} | Intention: {Message['intention']} | T: {Message['time']}")
        
    def receive_sebs_message(self, message):
        """
        Handles a received sebs message, updating intention table and believed node idleness
        """
        rospy.loginfo(f"- COMMS - Received SEBS message from: ID {message['source']}: ")
        rospy.loginfo(f"Position: {message['position']} | Intention: {message['intention']} | T: {message['time']}")
        self.intention_table[message["source"]] = message["intention"]
        self.node_idleness[message["position"]] = message["time"]
            
    def arrived_at_node(self):
        """
        Actions for an agent to take upon arriving at a set node (current_goal)
            Called at successful actionclient goal complete from Main.nav_cb
            Updates idleness and sends a sebs message
            Calls relevant patrol method
        """
        arrival_time = time.time()
        current_node = self.current_goal
        rospy.loginfo(f"- GOAL - Arrived at node: {current_node}")
        self.node_idleness[current_node] = arrival_time
        
        if self.patrol_method == "SEBS":
            self.sebs()
        elif self.patrol_method == "CGG":
            self.cgg()
            
        self.send_sebs_msg(current_node, arrival_time)
        
    def calculate_node_idleness(self, node, current_time):
        """
        Returns the idleness of requested node in seconds
        """
        return current_time - self.node_idleness[node]
        
        
    def sebs(self, gain1=0.1, gain2=20.0, edge_min=30.0):
        """
        State Exchange Bayesian Strategy (D.Portugal)
            Sets self.current_goal
            Built off of zaks implementation in py_patrol :) 
            @param gain1: A float representing the gain factor.
            @param gain2: A float representing the gain threshold.
            @param edge_min: A float representing the minimum edge weight.
        """
        
        
        
        pass
    
    def cgg(self):
        """
        Cyclic Patrol
        Assumes path is outlined in order. Will go from node 0 to node 1 etc. 
        If you want to use this one make sure to create the waypoint file accordingly
            Sets self.current_goal
        """
        self.current_goal += 1
        
        if self.current_goal == len(self.node_list):
            self.current_goal = 0
            
        if self.current_goal == self.start_node:
            rospy.loginfo("- CGG - Lap complete, repeating...")
            
        rospy.loginfo(f"- CGG - New goal node: {self.current_goal}")
        
            
        
        
        
    
    
        
    