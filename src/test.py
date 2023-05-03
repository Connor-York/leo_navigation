#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

def spin_robot():
    # Initialize ROS node
    rospy.init_node('spin_robot', anonymous=True)

    # Create a Twist publisher to send velocity commands to the robot
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Spin robot on the spot for 10 seconds
    t_end = rospy.Time.now() + rospy.Duration(7.5)
    while rospy.Time.now() < t_end:
        vel_msg = Twist()
        vel_msg.angular.z = 7.0
        vel_pub.publish(vel_msg)

    # Stop the robot
    vel_msg = Twist()
    vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        spin_robot()
    except rospy.ROSInterruptException:
        pass
