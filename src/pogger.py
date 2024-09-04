#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
import time 
import datetime
import rospkg 
import csv
import tf.transformations 

#getting date for saving
# current_date = datetime.date.today()
# formatted_date = current_date.strftime("%Y-%m-%d")
current_time_save = datetime.datetime.now()
current_time_save = current_time_save.strftime("%H:%M:%S")

#start time for comparison
start_time = time.time()

#getting csv paths for both the logs
rp = rospkg.RosPack()
package_path = rp.get_path('leo_navigation')

name = rospy.get_param("/pogger/robot_name")
trial_no = rospy.get_param("/pogger/trial_no")
trial_no = str(trial_no)
scenario = rospy.get_param("/pogger/trial_scenario")


csv_name = (name + "_" + scenario + "_" + trial_no)
    
vel_path = (package_path + "/logs/VEL_"  + csv_name + "_" + current_time_save + ".csv")
pose_path = (package_path + "/logs/POSE_"  + csv_name + "_" + current_time_save + ".csv")
battery_path = (package_path + "/logs/BATT_"  + csv_name + "_" + current_time_save + ".csv")

initial_x = rospy.get_param("/amcl/initial_pose_x")
initial_y = rospy.get_param("/amcl/initial_pose_y")
initial_a = rospy.get_param("/amcl/initial_pose_a")
flag = 0
if flag == 0
    current_pose = []
    flag = 1


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
    current_pose = [x,y,ox,oy,oz,ow,timestamp]


def battery_callback(msg):
    timestamp = time.time() - start_time
    data = [msg.data, timestamp]
    save_to_csv(battery_path,data)

def save_to_csv(csv_path,data):
    with open(csv_path, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(data)

if __name__ == "__main__":
    rospy.init_node("pogger") # init node (pose logger) 
    #start subscribers for both velocity and position
    #vel_subscriber = rospy.Subscriber("nav_vel",Twist,vel_callback)

    pose_subscriber = rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,pose_callback)

    #battery_subscriber = rospy.Subscriber("firmware/battery_averaged",Float32,battery_callback)

    rate = rospy.Rate(1) # 1 hz 
    while not rospy.is_shutdown():
        save_to_csv(pose_path,current_pose)
        rate.sleep()

    #rospy.spin()
