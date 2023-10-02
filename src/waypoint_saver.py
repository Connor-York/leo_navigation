#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import PointStamped
import csv
import rospkg


 # gets csv file path
rp = rospkg.RosPack()
package_path = rp.get_path('leo_navigation')
CSV_path = (package_path + "/waypoints/EXPERIMENT_NEW.csv")


def callback(msg):
        with open(CSV_path, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([msg.point.x, msg.point.y, 0])

def listener():
    rospy.init_node('waypoint_saver', anonymous=True)
    rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
