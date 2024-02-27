#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import csv
import rospkg


 # gets csv file path
rp = rospkg.RosPack()
package_path = rp.get_path('leo_navigation')
CSV_path = (package_path + "/waypoints/full_office_test.csv")


def callback(msg):
        with open(CSV_path, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([msg.point.x, msg.point.y, 0])

clicked_poses = []

def clicked_point_callback(clicked_point):
    global clicked_poses

    # Extract the x and y values from the clicked_point message
    x = clicked_point.point.x
    y = clicked_point.point.y

    # Create a Pose message with z and orientation set to 0
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0  # Assuming no rotation

    # Append the pose to the list
    clicked_poses.append(pose)

    # Create a PoseArray message
    pose_array = PoseArray()
    pose_array.header = Header()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = "map"  # Change the frame_id as needed

    # Append all the stored poses to the PoseArray
    pose_array.poses = clicked_poses

    # Publish the PoseArray
    pose_array_publisher.publish(pose_array)

# def listener():
#     rospy.init_node('waypoint_saver', anonymous=True)
#     rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, callback)
#     rospy.spin()

if __name__ == '__main__':
    # listener()

    rospy.init_node('waypoint_saver', anonymous=True)

    rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, callback)

    rospy.Subscriber("/clicked_point", Point, clicked_point_callback)

    # Create a publisher for the PoseArray
    pose_array_publisher = rospy.Publisher("/pose_array", PoseArray, queue_size=10)


    rospy.spin()
