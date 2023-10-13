#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
import time 
import datetime
import rospkg 
import csv
import os


#getting date for saving
current_date = datetime.date.today()
formatted_date = current_date.strftime("%Y-%m-%d")
current_time_save = datetime.datetime.now()
current_time_save = current_time_save.strftime("%H:%M:%S")

#start time for comparison
start_time = time.time()

#getting csv paths for both the logs
rp = rospkg.RosPack()
packagePath = rp.get_path('arLogger')
logFolder = os.path.join(packagePath, "logs")

#grabbing time and date to provide unique ID for logs
dateTime = datetime.now()
timenow = dateTime.strftime("%Y%m%d%H%M%S") #ISO 8601 Standard
    
velFullpath = os.path.join(logFolder, timenow + "_vellog.csv")
poseFullpath = os.path.join(logFolder, timenow + "_poselog.csv")
batFullpath = os.path.join(logFolder, timenow + "_batlog.csv")


def vel_callback(msg):
    lin_x = msg.linear.x
    ang_z = msg.angular.z
    timestamp = time.time() - start_time
    data = [lin_x,ang_z,timestamp]
    save_to_csv(velFullpath,data)

def pose_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w
    timestamp = time.time() - start_time
    data = [x,y,ox,oy,oz,ow,timestamp]
    save_to_csv(poseFullpath,data)

def battery_callback(msg):
    timestamp = time.time() - start_time
    data = [msg.data, timestamp]
    save_to_csv(batFullpath,data)

def save_to_csv(csv_path,data):
    with open(csv_path, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(data)

if __name__ == "__main__":
    rospy.init_node("pogger") # init node (pose logger)


    #start subscribers for both velocity and position
    vel_subscriber = rospy.Subscriber("nav_vel",Twist,vel_callback)

    pose_subscriber = rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,pose_callback)

    battery_subscriber = rospy.Subscriber("firmware/battery_averaged",Float32,battery_callback)

    rospy.spin()
