#!/usr/bin/env python3
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospkg
import math



def robot_locator_cb(msg):
    for marker in msg.markers: 
        x = marker.pose.pose.position.x
        y = marker.pose.pose.position.y
        dist = math.sqrt(x**2 + y**2)
        print(marker.id)
        print(dist)







if __name__ == "__main__":

    rospy.init_node('robot_locator_test')
    robot_locator = rospy.Subscriber("ar_pose_marker", AlvarMarkers, robot_locator_cb)

    rospy.spin()