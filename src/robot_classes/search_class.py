#!/usr/bin/env python3

import rospy

class Searcher:
    
    def __init__(self):
        self.search_method = rospy.get_param("~search_method")
    
    def PSO_step(self):
        pass
        
    def ECOLI_step(self):
        pass
    
    def HYBRID_step(self):
        pass

