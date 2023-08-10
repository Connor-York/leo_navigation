#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import time 
import datetime
import rospkg 
import csv

#getting date for saving
current_date = datetime.date.today()
formatted_date = current_date.strftime("%Y-%m-%d")
current_time_save = datetime.datetime.now()
current_time_save = current_time_save.strftime("%H:%M:%S")

#start time for comparison
start_time = time.time()

#getting csv paths for both the logs
rp = rospkg.RosPack()
package_path = rp.get_path('leo_navigation')
    
vel_path = (package_path + "/logs/VEL_" + formatted_date + "_" + current_time_save + ".csv")
pose_path = (package_path + "/logs/POSE_" + formatted_date + "_" + current_time_save + ".csv")


def vel_callback(msg):
    lin_x = msg.linear.x
    ang_z = msg.angular.z
    rospy.loginfo(lin_x)
    rospy.loginfo(ang_z)
    timestamp = time.time() - start_time
    data = [lin_x,ang_z,timestamp]
    save_to_csv(vel_path,data)

def pose_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    timestamp = time.time() - start_time
    data = [x,y,timestamp]
    save_to_csv(pose_path,data)

def save_to_csv(csv_path,data):
    with open(csv_path, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(data)

if __name__ == "__main__":
    rospy.init_node("pogger") # init node (pose logger)


    #start subscribers for both velocity and position
    vel_subscriber = rospy.Subscriber("nav_vel",Twist,vel_callback)

    pose_subscriber = rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,pose_callback)

    rospy.spin()
