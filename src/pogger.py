#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32, String
import time 
import datetime
import rospkg 
import csv
import socket
import os

#getting date for saving
current_date = datetime.date.today()
formatted_date = current_date.strftime("%Y-%m-%d")
current_time_save = datetime.datetime.now()
timenow = current_time_save.strftime("%Y%m%d%H%M%S")

#getting hostName to determine different
hostName = str(socket.gethostname())

#start time for comparison
start_time = time.time()

timeThresholdLow = rospy.get_param("/timeThresholdLow")
timeThresholdHigh = rospy.get_param("/timeThresholdHigh")

print (timeThresholdLow)

#getting csv paths for both the logs
rp = rospkg.RosPack()
package_path = rp.get_path('arLogger')

folder_path = (package_path + "/logs/" + hostName + "/TTH_" + str(timeThresholdLow) + "_" + str(timeThresholdHigh))


if not os.path.exists(folder_path):
    os.makedirs(folder_path)
    
vel_path = (folder_path + "/" + timenow + hostName + "_TTH_" + str(timeThresholdLow) + "_" + str(timeThresholdHigh) + "_vellog.csv")
pose_path = (folder_path + "/" + timenow + hostName + "_TTH_" + str(timeThresholdLow) + "_" + str(timeThresholdHigh) + "_poselog.csv")
battery_path = (folder_path + "/" + timenow + hostName + "_TTH_" + str(timeThresholdLow) + "_" + str(timeThresholdHigh) + "_batlog.csv") 
metrics_path = (folder_path + "/" + timenow + hostName + "_TTH_" + str(timeThresholdLow) + "_" + str(timeThresholdHigh) + "_metricspog.csv") 

def vel_callback(msg):
    lin_x = msg.linear.x
    ang_z = msg.angular.z
    timestamp = time.time() - start_time
    data = [lin_x,ang_z,timestamp]
    save_to_csv(vel_path,data)

def pose_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w
    timestamp = time.time() - start_time
    data = [x,y,ox,oy,oz,ow,timestamp]
    save_to_csv(pose_path,data)

def battery_callback(msg):
    timestamp = time.time() - start_time
    data = [msg.data, timestamp]
    save_to_csv(battery_path,data)

def metrics_callback(msg):
    data = msg.data.split(',')
    save_to_csv(metrics_path, data)

def save_to_csv(csv_path,data):
    with open(csv_path, "a", newline="") as file:
        writer = csv.writer(file, quoting=csv.QUOTE_NONE)
        writer.writerow(data)

if __name__ == "__main__":
    rospy.init_node("pogger") # init node (pose logger)

    


    #start subscribers for both velocity and position
    vel_subscriber = rospy.Subscriber("nav_vel",Twist,vel_callback)

    pose_subscriber = rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,pose_callback)

    battery_subscriber = rospy.Subscriber("firmware/battery_averaged",Float32,battery_callback)

    metrics_subscriber = rospy.Subscriber("metricsTopic", String, metrics_callback)

    rospy.spin()
