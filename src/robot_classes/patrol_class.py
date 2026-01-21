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
import random

class Patroller:
    
    def __init__(self, id, package_path, server_pub, num_robots, start_time):
        
        self.id = id
        
        # Initialise Patrolling 
        patrol_route = package_path + "/waypoints/" + rospy.get_param("~patrol_route")
        unit_adjacency = package_path + "/waypoints/" + rospy.get_param("~patrol_adjacency")
        
        #Generate waypoint list and adjacency matrices
        self.node_list = self.waypoint_gen(patrol_route)
        self.adjacency, self.node_neighbours = self.gen_adjacencies(unit_adjacency, patrol_route)
        
        self.start_node = rospy.get_param("~start_node")
        self.current_goal = self.start_node
        self.patrol_method = rospy.get_param("~patrol_method")
        if self.patrol_method not in ["CGG","SEBS"]:
            rospy.logfatal(f"Patrol method: {self.patrol_method} not recognised.")
            rospy.signal_shutdown(f"Invalid patrol method.")
            
        self.intention_table = np.full(num_robots, -1, dtype=int)
        self.node_idleness = np.full(len(self.node_list), start_time, dtype=np.float64) # node_idleness array is the last time each node was visited 
        self.sent_log = [] # curr_time , message['position'], message['intention']
        self.rec_log = [] # curr_time, message['source'], message['time'], message['position'], message['intention']
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
            
            #rospy.loginfo(f"Going to: {Quaternion(*(quaternion_from_euler(0, 0, yawangle, axes='sxyz')))} <=================================")
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            # Exploit n variable to cycle in quat_seq
            pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
            n += 1
            
        return pose_seq
    
    
    def gen_adjacencies(self, unit_adj, waypoint_csv):
        """
        Takes unit_adjacency txt and returns it as array, and generates a full adjacency matrix with weights
        """
        waypoints = np.loadtxt(waypoint_csv, delimiter=',')
    
        with open(unit_adj) as file:
            unit_adj = list()
            for line in file:            
                unit_adj.append(np.fromstring(line, sep=',', dtype=int))
    
        adj_matrix = np.zeros((len(waypoints),len(waypoints)))
        for node, item in enumerate(unit_adj):
            node_coordinates = np.array([ waypoints[node][0] , waypoints[node][1] ])
            for neighbour in item:
                neighbour_coordinates = np.array([ waypoints[neighbour][0] , waypoints[neighbour][1] ])
                adj_matrix[node][neighbour] = np.linalg.norm(node_coordinates - neighbour_coordinates)
        
        return adj_matrix, unit_adj

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
    
    def return_to_patrol(self, pose_x, pose_y):
        closest_node_dist = 99999
        closest_node_id = None
        for count, node in enumerate(self.node_list):
            dist = np.linalg.norm(np.array([pose_x, pose_y]) - np.array([node.position.x,node.position.y]))
            if dist < closest_node_dist:
                closest_node_id = count
                closest_node_dist = dist
        
        if closest_node_id == None:
            self.current_goal = np.random.randint(len(self.node_list))
        else:
            self.current_goal = closest_node_id
        
        return self.set_nav_goal()
        
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
        self.sent_log.append([arrival_time, Message['position'], Message['intention']])
        #rospy.loginfo(f"- COMMS - Sebs message sent: ")
        #rospy.loginfo(f"Position: {Message['position']} | Intention: {Message['intention']} | T: {Message['time']}")
        
        
    def receive_sebs_message(self, message, curr_time):
        """
        Handles a received sebs message, updating intention table and believed node idleness
        """
        rospy.loginfo(f"- COMMS - Received SEBS message from: ID {message['source']}: ")
        rospy.loginfo(f"Position: {message['position']} | Intention: {message['intention']} | T_msg: {message['time']} | T_now: {curr_time}")
        self.intention_table[message["source"]] = message["intention"]
        if message["position"] is not None:
            self.node_idleness[message["position"]] = message["time"]
        self.rec_log.append([curr_time, message['source'], message['time'], message['position'], message['intention']])
        self.idlenesslog.append([curr_time, self.current_goal, self.node_idleness.copy()])
            
    def arrived_at_node(self, curr_time):
        """
        Actions for an agent to take upon arriving at a set node (current_goal)
            Called at successful actionclient goal complete from Main.nav_cb
            Updates idleness and sends a sebs message
            Calls relevant patrol method
        """
        
        current_node = self.current_goal
        rospy.loginfo(f"- GOAL - Arrived at node: {current_node}")
        self.node_idleness[current_node] = curr_time
        
        if self.patrol_method == "SEBS":
            self.current_goal = self.sebs(current_node, curr_time)
        elif self.patrol_method == "CGG":
            self.current_goal = self.cgg(current_node)
        
        self.send_sebs_msg(current_node, curr_time)
        self.idlenesslog.append([curr_time, self.current_goal, self.node_idleness.copy()])
        
        return self.set_nav_goal()
        
        
    def calculate_node_idleness(self, node, current_time):
        """
        Returns the idleness of requested node in seconds
        """
        return current_time - self.node_idleness[node]
        
        
    def count_intentions(self, node):
        """
        Takes node number and returns number of intentions going there
        """
        count = 0
        for intention in self.intention_table:
            if intention == node:
                count += 1 
        return count
        
    def sebs(self, current_node, current_time, gain1=0.1, gain2=50.0, edge_min=1.0):
        """
        State Exchange Bayesian Strategy (D.Portugal)
            Sets self.current_goal
            Built off of zaks implementation in py_patrol which is built off patrol_sim implementation by D.Portugal
            @param gain1: A float representing the gain factor.
            @param gain2: A float representing the gain threshold.
            @param edge_min: A float representing the minimum edge weight (to avoid inflated gain on really short distances, 
                                                                            set to minimum edge distance or ~0.5 maximum edge)
        """
        neighbours = self.node_neighbours[current_node]
        # rospy.loginfo(f"- SEBS - Neighbours: {[ (neighbour, self.calculate_node_idleness(neighbour, current_time)) for neighbour in neighbours ]}")
        
        if len(neighbours) > 1:
            log_result = math.log(1.0/gain1)
            posterior_probability = np.zeros(len(neighbours))
            for i in range(len(neighbours)):
                
                neighbour_id = neighbours[i]
                neighbour_edge_weight1 = self.adjacency[current_node][neighbour_id]
                neighbour_edge_weight = max(neighbour_edge_weight1, edge_min)
                gain = self.calculate_node_idleness(neighbour_id, current_time) / neighbour_edge_weight
                
                if gain < gain2:
                    exp_param = (log_result/gain2) * gain
                    posterior_probability[i] = gain1 * math.exp(exp_param)
                    
                else:
                    posterior_probability[i] = 1.0
                
                count = self.count_intentions(neighbour_id)
                
                if count >0:
                    num_agents = len(self.intention_table)
                    p_gain_state = math.pow(2, num_agents - count) / (math.pow(2, num_agents) - 1.0)
                    posterior_probability[i] *= p_gain_state
                
                rospy.loginfo(f"- SEBS - Neighbour: {neighbour_id} | Idleness: {self.calculate_node_idleness(neighbour_id, current_time):.2f} | Edge weight: {neighbour_edge_weight1:.2f}, {neighbour_edge_weight:.2f} | Gain: {gain:.2f} | Intentions: {count} | Posterior prob: {posterior_probability[i]:.4f}")
            # Choose the one in the posterior probability with the largest value
            # return a numpy array, and if there are more than one include the index that each are found at
            # Return the 0th element of the tuple, as np.where returns a tuple
            max_posterior_prob_index = np.where(posterior_probability == np.max(posterior_probability))[0]
            number_of_max_probs = len(max_posterior_prob_index)
            # If there are more than 1 max_probs, choose randomly
            if number_of_max_probs > 1:
                # generate random int between 0 - number_of_max_probs
                random_index = random.randint(0, number_of_max_probs-1)         # Not including total length
                # Choose randomly from the array of elements with the highest posterior_probability
                next_goal = neighbours[max_posterior_prob_index[random_index]]
            else:
                # If there is only one choice of max_posterior_prob, use this value
                next_goal = neighbours[max_posterior_prob_index[0]]
                
        else:
            # if only one go there
            next_goal = neighbours[0]

        rospy.loginfo(f"- SEBS - New goal node: {next_goal}")
        return int(next_goal)
    
    def cgg(self, current_node):
        """
        Cyclic Patrol
        Assumes path is outlined in order. Will go from node 0 to node 1 etc. 
        If you want to use this one make sure to create the waypoint file accordingly
            Returns next_goal (to be set as self.current_goal)
        """
        next_goal = current_node + 1
        
        if next_goal == len(self.node_list):
            next_goal = 0
            
        if next_goal == self.start_node:
            rospy.loginfo("- CGG - Lap complete, repeating...")
            
        rospy.loginfo(f"- CGG - New goal node: {next_goal}")
        
        return next_goal
            
        
        
        
    
    
        
    
