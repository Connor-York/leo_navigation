#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseGoal


#Service
from signal_detection.srv import GetSignalData #TODO: add this to the dependencies
    
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
    new_yaw = normalise_angle(new_yaw + np.pi)
    return new_yaw
    
    

class Searcher:
    
    def __init__(self, step_distance):
        self.search_method = rospy.get_param("~search_method")
        self.pose_x = rospy.get_param("/amcl/initial_pose_x")
        self.pose_y = rospy.get_param("/amcl/initial_pose_y")
        self.pose_yaw = rospy.get_param("/amcl/initial_pose_a")
        self.rssi_avg = -999
        
        self.rssi_previous = self.rssi_avg # For ECOLI
        
        self.step_distance = step_distance
        
    def read_signal(self):
        """
        Reads signal data from get_signal_data service (running from signal_detection package)
        """  
        rospy.wait_for_service('get_signal_data')
        try:
            get_data = rospy.ServiceProxy('get_signal_data', GetSignalData)
            resp = get_data()
            rospy.loginfo(resp)
            self.pose_x = resp.x
            self.pose_y = resp.y
            self.pose_yaw = resp.yaw
            self.rssi_avg = resp.avg_rssi
        
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            
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
        
        rospy.loginfo(f"- GOAL - Navigating to x: {x}, y: {y}, yaw: {yaw}")

        return goal

    
    def PSO_step(self):
        pass
        
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
        if self.rssi_avg < self.rssi_previous: #maybe add a threshold here i.e. if its 5 rssi worse or smth
            new_yaw = random_turn(self.pose_yaw, 180) # turn rand +/- 180deg
            rospy.loginfo("WORSE")
        else:
            new_yaw = random_turn(self.pose_yaw, 5) # turn rand +/- 5deg
        
        new_x = self.pose_x + (self.step_distance * np.cos(new_yaw))
        new_y = self.pose_y + (self.step_distance * np.sin(new_yaw))
        

        rospy.loginfo(f"Prev: {self.rssi_previous},{self.pose_yaw} new {self.rssi_avg},{new_yaw}")
        self.rssi_previous = self.rssi_avg
        
        #TODO Update new intention to previous etc. 
        
    
    def HYBRID_step(self):
        pass

