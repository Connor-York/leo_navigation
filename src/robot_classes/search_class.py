#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseGoal
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import json
import time
import ast
#from costmap_2d import Costmap2DROS

#Service
from signal_detection.srv import GetSignalData #TODO: add this to the dependencies
from leo_navigation.srv import CheckPoseCollision


# Utility Functions
def normalise_angle(angle):
    """
    Normalise angle to [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def random_turn(current_yaw, max_degrees):
    """
    Returns a yaw in radians randomly changed by +/- max_degrees (deg) from current_yaw (radians)
    """
    
    max_radians = np.deg2rad(max_degrees)
    delta = np.random.uniform(-max_radians, max_radians)
    new_yaw = current_yaw + delta
    new_yaw = normalise_angle(new_yaw)
    return new_yaw
    
    
# Main Class
class Searcher:
    
    def __init__(self, step_distance, num_agents, id, server_pub, start_time):
        
        self.id = id
        self.search_method = rospy.get_param("~search_method")
        self.bad_diff = rospy.get_param("~bad_diff")
        self.pose_x = rospy.get_param("/amcl/initial_pose_x")
        self.pose_y = rospy.get_param("/amcl/initial_pose_y")
        self.pose_yaw = rospy.get_param("/amcl/initial_pose_a")
        # pose_str = rospy.get_param("/slam_toolbox/map_start_pose")
        # pose_list = ast.literal_eval(pose_str)
        # self.pose_x = pose_list[0]
        # self.pose_y = pose_list[1]
        # self.pose_yaw = pose_list[2]
        self.rssi_avg = -999
        self.rssi_raw = -999
        
        # For publishing teammate obstacles
        #self.teammate_obstacle_pub = rospy.Publisher('/teammate_obstacles', PointCloud2, queue_size=1)
        self.teammate_radius = 0.3  # radius around each robot to mark
        self.obstacle_resolution = 0.05  # spacing between points
        
        # For ECOLI
        self.rssi_previous = self.rssi_avg
        # For PSO
        self.p_best = (self.rssi_avg, (self.pose_x, self.pose_y)) # value, position (x,y)
        self.g_best = (self.rssi_avg, (self.pose_x, self.pose_y)) # value, position (x,y)
        self.isbest = True                     # Whether the g_best was found by this agent
        self.velocity = np.zeros(2)
        self.repulsion_strength = rospy.get_param("~repulsion_strength")
        self.c1 = rospy.get_param("~c1")
        self.c2 = rospy.get_param("~c2")
        self.w = rospy.get_param("~w")
        self.current_goal = None # To store current goal for re-attempting if failed
        
        self.agent_positions = [None] * num_agents
        self.source_found = (False, None) # Bool: Found? ID of agent that found it
        self.last_gbest_update = start_time
        self.source_found_log = None
        self.search_log = [] # time, method, other data depending on method
        
        self.step_distance = step_distance
        self.server_pub = server_pub # For sending communications
        
        if rospy.get_param("~delay_signal_start"): # if waiting for signal start message, signal_start is false (set to true in read_inbox)
            self.signal_start = False
        else:
            self.signal_start = True
        
        #TODO add these services stuff to dependencies and add timeout for them not connecting with a failure
        
        rospy.wait_for_service('get_signal_data')
        self.get_signal_data = rospy.ServiceProxy('get_signal_data', GetSignalData)
        
        rospy.wait_for_service("check_pose_collision")
        self.check_collision = rospy.ServiceProxy("check_pose_collision", CheckPoseCollision)
        
        self.read_signal(None, True) # update initial parameters 
        
    def read_signal(self, current_state, measure):
        """
        Reads signal data from get_signal_data service (running from signal_detection package)
        also updates robot pose 
        """  
        new_state = None
        try:
            
            resp = self.get_signal_data()
            self.pose_x = resp.x
            self.pose_y = resp.y
            self.pose_yaw = resp.yaw
            
            if self.signal_start and measure:    
                self.rssi_avg = resp.avg_rssi
                self.rssi_raw = resp.rssi_val
            
                if self.source_found[0] == False:
                    # start search if first reading
                    if current_state == 'patrolling' and self.rssi_avg > -999:
                        new_state = 'searching'
                    # update personal and global best
                    if self.rssi_avg > self.p_best[0]:
                        self.p_best = (self.rssi_avg, (self.pose_x,self.pose_y))
                        if self.rssi_avg > self.g_best[0]:
                            rospy.loginfo(f"- SEARCH - Updating GBest from {self.g_best} to {self.p_best} (rssi_avg)")
                            self.g_best = self.p_best
                            self.isbest = True
                            self.last_gbest_update = time.time()

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        return new_state
        
    def send_signal_message(self, curr_time):
        Message = { 
            'source':self.id,
            'type':"signal",
            'source_found':self.source_found,
            'position':(self.pose_x, self.pose_y), 
            'g_best':self.g_best,
            't_sent':curr_time
        }
        self.server_pub.publish(json.dumps(Message))
        #rospy.loginfo(f"- COMMS - Signal message sent, t={curr_time}")
        #rospy.loginfo(f"- COMMS - Signal message sent")
        
    def receive_signal_message(self, message, current_state):
        new_state = None
        
        #rospy.loginfo(f"- COMMS - Signal message received from ID {message['source']}")
        #rospy.loginfo(f"- COMMS - Signal message received, received t={current_time}, sent t={message['t_sent']} | diff = {current_time-message['t_sent']}")
        current_time = time.time()
        #rospy.loginfo(f"- RAW - Signal Sent: {message['raw_time']} Received: {now} | diff {now-message['raw_time']}")
        
        if message['g_best'][0] > self.g_best[0]:
            rospy.loginfo(f"- COMMS - Updating GBest from {self.g_best} to {message['g_best']} (from robot {message['source']})")
            if self.source_found[0] == True:
                rospy.logerr("ERROR: SOURCE FOUND BUT RECEIVED BETTER GBEST FROM ANOTHER ROBOT ===========================")
                rospy.loginfo(f"current gbest: {self.g_best}, received gbest: {message['g_best']}, source found: {self.source_found}, robot id: {self.id}, other robot id: {message['source']}")
                rospy.logerr("===========================================================================================")
            self.g_best = message['g_best']
            self.isbest = False
            self.last_gbest_update = current_time
            if current_state == 'patrolling':
                new_state = 'searching'
        
        self.agent_positions[message['source']] = message['position']
        
        if message['source_found'][0] == True and self.source_found[0] == False:
            self.source_found = message['source_found']
            self.source_found_log = [current_time, message['source_found'][1], 'message']
            if current_state == 'searching':
                new_state = 'patrolling'
        
        return new_state
    
    # def publish_teammate_obstacles(self):
    #     """
    #     Publishes circles around known teammate positions as obstacles
    #     """
    #     points = []
        
    #     for i, pos in enumerate(self.agent_positions):
    #         if pos is None or i == self.id:
    #             continue
    #         cx, cy = pos
    #         # Generate filled circle
    #         for r in np.arange(0, self.teammate_radius, self.obstacle_resolution):
    #             for theta in np.arange(0, 2*np.pi, self.obstacle_resolution / max(r, 0.01)):
    #                 px = cx + r * np.cos(theta)
    #                 py = cy + r * np.sin(theta)
    #                 points.append((px, py, 0.0))
        
    #     if points:
    #         header = Header(stamp=rospy.Time.now(), frame_id="map")
    #         cloud = pc2.create_cloud_xyz32(header, points)
    #         self.teammate_obstacle_pub.publish(cloud)
        
               
    def generate_nav_goal(self,x,y,yaw):
        """
        generates nav goal from x,y,yaw
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        quat = quaternion_from_euler(0, 0, yaw)
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = Quaternion(*quat)  # Unpack quaternion
        
        #rospy.loginfo(f"- GOAL - Navigating to x: {x}, y: {y}, yaw: {yaw}")

        return goal
    
    
    def calculate_repulsion(self, my_position):
        forces = np.zeros([len(self.agent_positions), 2], dtype=float)

        for count, agent_pos in enumerate(self.agent_positions):
            if agent_pos is not None:
                if count == self.id: 
                    continue
                direction = my_position - agent_pos
                distance = np.linalg.norm(direction)
                if distance == 0.0:
                    # if ontop of it, choose a random direction to go in 
                    random_vector = np.random.uniform(-1,1,2)
                    unit_direction = random_vector / np.linalg.norm(random_vector)
                    magnitude = self.repulsion_strength 
                else:
                    unit_direction = direction / distance
                    magnitude = max(self.repulsion_strength - distance, 0)  #self.repulsion_strength / distance**2
                
                agent_repulsion = unit_direction * magnitude
                forces[count] = agent_repulsion
                self.agent_positions[count] = None # clear (TODO change this to a queue i guess)

        
        repulsion_vector = np.sum(forces, axis=0)
        if np.linalg.norm(repulsion_vector) > 0:
            
            rospy.loginfo(f"################## BEING REPULSED ####################")
            rospy.loginfo(f"Repulsion Vector: {repulsion_vector} | Magnitude: {np.linalg.norm(repulsion_vector)}")
        return repulsion_vector
    
    
    def PSO_step(self):
        # rospy.loginfo(f"Current Position: {self.pose_x,self.pose_y}")
        # rospy.loginfo(f"PBEST: {self.p_best}")
        # rospy.loginfo(f"GBEST: {self.g_best}")
        r1, r2 = np.random.rand(2)
        my_position = np.array([self.pose_x, self.pose_y])
        repulsion_vector = self.calculate_repulsion(my_position)
        
        self.velocity = self.w * self.velocity + (self.c1 * r1 * (np.array(self.p_best[1]) - my_position)) + (self.c2 * r2 * (np.array(self.g_best[1]) - my_position))
        dist = np.linalg.norm(self.velocity)
        #rospy.loginfo(f"Velocity Vector before repulsion: {self.velocity} | Magnitude: {dist}")
        if dist > self.step_distance:
            velocity_clamped = self.velocity * (self.step_distance / dist)
        else:
            velocity_clamped = self.velocity

        velocity_clamped = velocity_clamped + repulsion_vector 
        #rospy.loginfo(f"Velocity Vector after repulsion: {velocity_clamped} | Magnitude: {np.linalg.norm(velocity_clamped)}")
        
        new_position = my_position + velocity_clamped
        movement_vector = new_position - my_position
        new_yaw = np.arctan2(movement_vector[1], movement_vector[0])
        
        self.search_log.append([
            time.time(), 'PSO',
            self.pose_x, self.pose_y, self.pose_yaw,
            new_position[0], new_position[1], new_yaw,
            self.velocity[0], self.velocity[1],
            velocity_clamped[0], velocity_clamped[1],
            repulsion_vector[0], repulsion_vector[1]
        ])
        
        return self.find_safe_goal(new_position[0], new_position[1], new_yaw)
        

        
    def ECOLI_step(self):
        """
        Ecoli chemotaxis from R.A. Russell et. al 2003
            Different to my implementation in py patrol which was turn guaranteed 180 if worse
            continue if same or better,
            and then for both add a 50% chance to go +/- 45 ontop
        PSEUDOCODE:
            if current sensor reading is an improvement on
            previous sensor reading:
                rotate ± random(5◦) and move forward m
                units
            else:
                rotate ± random(180◦) and move forward m
                units 
        """
        rospy.loginfo(f"== IN ECOLI === Prev: {self.rssi_previous},New: {self.rssi_avg}")
        rospy.loginfo(f" GBEST: {self.g_best}")
        if self.rssi_avg < self.rssi_previous - self.bad_diff: 
            new_yaw = random_turn(self.pose_yaw, 180) # turn rand +/- 180deg
            val = 'bad'
        else:
            new_yaw = random_turn(self.pose_yaw, 5) # turn rand +/- 5deg
            val = 'good'
        
        new_x = self.pose_x + (self.step_distance * np.cos(new_yaw))
        new_y = self.pose_y + (self.step_distance * np.sin(new_yaw))
        self.search_log.append([
            time.time(), 'ECOLI',
            self.pose_x, self.pose_y, self.pose_yaw,
            new_x, new_y, new_yaw,
            self.rssi_previous, self.rssi_avg, val
        ])
        self.rssi_previous = self.rssi_avg
        # check if its safe
        return self.find_safe_goal(new_x, new_y, new_yaw)
        
    def search_step(self):
        """
        Chooses and performs a search step based on the selected search method
        returns goal
        """
        if self.search_method == 'PSO':
            return self.PSO_step()
        elif self.search_method == 'ECOLI':
            return self.ECOLI_step()
        elif self.search_method == 'HYBRID':
            if self.isbest:
                rospy.loginfo(f"Using ECOLI step (isbest={self.isbest})")
                return self.ECOLI_step()
            else:
                rospy.loginfo(f"Using PSO step (isbest={self.isbest})")
                return self.PSO_step()

    def find_goal_through_obstacle(self, start_x, start_y, direction_yaw, step_distance, max_raycast_distance=3.0, raycast_resolution=0.3):
        """
        If the immediate goal is blocked, raycast forward to find a free point
        on the other side of the obstacle.
        
        Args:
            start_x, start_y: Current position
            direction_yaw: Intended direction of travel
            step_distance: Desired step distance (used as minimum search distance)
            max_raycast_distance: Maximum distance to search along ray
            raycast_resolution: Distance between collision checks along ray
        
        Returns:
            goal if found, None if no valid point found along ray
        """
        # Start checking from step_distance, incrementing by resolution
        check_distance = step_distance
        
        dx = np.cos(direction_yaw)
        dy = np.sin(direction_yaw)
        
        checks_made = 0
        max_checks = int((max_raycast_distance - step_distance) / raycast_resolution) + 1
        
        while check_distance <= max_raycast_distance and checks_made < max_checks:
            test_x = start_x + check_distance * dx
            test_y = start_y + check_distance * dy
            
            goal = self.generate_nav_goal(test_x, test_y, direction_yaw)
            resp = self.check_collision(goal.target_pose)
            checks_made += 1
            
            if resp.is_free:
                rospy.loginfo(f"Found free point at distance {check_distance:.2f}m after {checks_made} checks")
                self.search_log.append([time.time(), 'RAYCAST', check_distance, checks_made])
                return goal
            
            check_distance += raycast_resolution
        
        rospy.loginfo(f"No free point found along ray after {checks_made} checks (max dist: {max_raycast_distance}m)")
        return None


    def find_safe_goal(self, new_x, new_y, new_yaw):
        """
        Wrapper that tries raycast first, then falls back to rotation.
        Returns a valid goal.
        """
        goal = self.generate_nav_goal(new_x, new_y, new_yaw)
        resp = self.check_collision(goal.target_pose)
        
        if resp.is_free:
            return goal
        
        # Try raycasting through the obstacle
        raycast_goal = self.find_goal_through_obstacle(
            self.pose_x, self.pose_y, new_yaw,
            step_distance=self.step_distance,
            max_raycast_distance=5.0,  # Tune this based on your environment
            raycast_resolution=0.3     # ~10 checks max for 3m range
        )
        
        if raycast_goal is not None:
            return raycast_goal
        
        # Fallback: rotation method (for corners or external walls)
        rospy.loginfo("Raycast failed, falling back to rotation")
        return self.find_goal_by_rotation(new_yaw)


    def find_goal_by_rotation(self, starting_yaw):
        """
        Rotation fallback - try 45 degree increments.
        Limited to 7 attempts (315 degrees) to avoid infinite loops.
        """
        new_yaw = starting_yaw
        turn_ratio = 16 # 360/turn_ratio degrees per attempt
        
        for attempt in range(turn_ratio-1):  # -1 because the last resort is to stay in place
            new_yaw = normalise_angle(new_yaw - ((2*np.pi)/turn_ratio))
            new_x = self.pose_x + self.step_distance * np.cos(new_yaw)
            new_y = self.pose_y + self.step_distance * np.sin(new_yaw)
            
            goal = self.generate_nav_goal(new_x, new_y, new_yaw)
            resp = self.check_collision(goal.target_pose)
            
            if resp.is_free:
                rospy.loginfo(f"Rotation found safe pose after {attempt + 1} attempts")
                self.search_log.append([time.time(), 'ROTATION', attempt + 1])
                return goal
        
        # Last resort: stay in place (or handle error)
        rospy.logwarn("**********No safe goal found in any direction!")
        return self.generate_nav_goal(self.pose_x, self.pose_y, new_yaw)