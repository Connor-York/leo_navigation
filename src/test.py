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

    # Create a MoveBase client to send goal requests to move_base
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    # Set up goal position and orientation for move_base
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.w = 1.0

    # Spin robot on the spot for 10 seconds
    t_end = rospy.Time.now() + rospy.Duration(10)
    while rospy.Time.now() < t_end:
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.position.z = 0
        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()
        vel_msg = Twist()
        vel_msg.angular.z = 1.0
        vel_pub.publish(vel_msg)

    # Stop the robot
    vel_msg = Twist()
    vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        spin_robot()
    except rospy.ROSInterruptException:
        pass
