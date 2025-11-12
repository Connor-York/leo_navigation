#!/usr/bin/env python3

import rospy

#Service
from signal_detection.srv import GetSignalData #TODO: add this to the dependencies
    

class Searcher:
    
    def __init__(self):
        self.search_method = rospy.get_param("~search_method")
        self.pose_x = rospy.get_param("/amcl/initial_pose_x")
        self.pose_y = rospy.get_param("/amcl/initial_pose_y")
        self.pose_yaw = rospy.get_param("/amcl/initial_pose_a")
        self.rssi_avg = -999
        
    def read_signal(self):
        """
        Reads signal data from get_signal_data service (running from signal_detection package)
        """  
        rospy.wait_for_service('get_signal_data')
        try:
            get_data = rospy.ServiceProxy('get_signal_data', GetSignalData)
            resp = get_data()
            #rospy.loginfo(resp)
            self.pose_x = resp.x
            self.pose_y = resp.y
            self.pose_yaw = resp.yaw
            self.rssi_avg = resp.avg_rssi
        
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
    def PSO_step(self):
        pass
        
    def ECOLI_step(self):
        pass
    
    def HYBRID_step(self):
        pass

