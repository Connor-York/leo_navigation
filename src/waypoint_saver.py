#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import csv
import rospkg
import sys
import tf

# Oh my god this code is bad 

 # gets csv file path
rp = rospkg.RosPack()
package_path = rp.get_path('leo_navigation')
file_arg = sys.argv[1]
CSV_path = (f"{package_path}/waypoints/{file_arg}.csv") 

marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=10)
markers = []

clicked_points = []

def save_file():
    rospy.loginfo("Saving and shutting down...")
    with open(CSV_path, "a", newline="") as file:
        writer = csv.writer(file)
        for point in clicked_points:
            x, y, theta = point
            writer.writerow([x, y, 0, theta])
            
def publish_marker(idx, pose, frame_id):
    global markers
    ma = MarkerArray()
    
    # Arrow
    arrow = Marker()
    arrow.header.frame_id = frame_id
    arrow.header.stamp = rospy.Time.now()
    arrow.ns = "waypoints"
    arrow.id = idx*2
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.pose = pose
    arrow.scale.x = 0.4  # arrow length
    arrow.scale.y = 0.1
    arrow.scale.z = 0.1
    arrow.color.r = 1.0
    arrow.color.g = 0.2
    arrow.color.b = 0.2
    arrow.color.a = 1.0
    
    # Text
    text = Marker()
    text.header.frame_id = frame_id
    text.header.stamp = rospy.Time.now()
    text.ns = "waypoints"
    text.id = idx*2 + 1
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position.x = pose.position.x
    text.pose.position.y = pose.position.y 
    text.pose.position.z = pose.position.z + 0.5
    text.scale.z = 0.4
    text.color.r = 0.0
    text.color.g = 0.0
    text.color.b = 0.0
    text.color.a = 1.0
    text.text = str(idx)
    
    markers.extend([arrow, text])
    ma.markers = markers
    marker_pub.publish(ma)


def goal_pose_callback(msg):
    global clicked_points
    
    x = msg.pose.position.x
    y = msg.pose.position.y
    
    # Convert quaternion to yaw (theta)
    orientation_q = msg.pose.orientation
    (_, _, yaw) = tf.transformations.euler_from_quaternion([
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    ])
    
    clicked_points.append([x, y, yaw])
    rospy.loginfo(f"Saved waypoint: x={x:.2f}, y={y:.2f}, theta={yaw:.2f}")
    
    # Publish marker for RViz
    publish_marker(len(clicked_points)-1, msg.pose, msg.header.frame_id)


if __name__ == '__main__':
    
    rospy.init_node('waypoint_saver', anonymous=True)
    rospy.on_shutdown(save_file)
    
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_pose_callback)

    rospy.loginfo(f"Setup done, save location: {file_arg}.csv, Subscribe to /waypoint_markers in RVIZ")

    rospy.spin()
