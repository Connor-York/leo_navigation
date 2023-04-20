#!/usr/bin/env python3
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math
import pandas as pd

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self,waypoints,theta):

        rospy.init_node('move_base_sequence')

       # df = pd.read_csv('~/ros_ws/src/leo_navigation/waypoints/waypoints_deskspace.csv', sep=',', header=None)
       # theta = list(df.loc[:,3].values)
       # wayp = df.loc[:,0:2]
       # waypoints = []
       # wayp = wayp.values.tolist()
       # print(wayp)
      #  for sublist in wayp:
      #    for item in sublist:
      #       waypoints.append(item)
        self.waypoints = waypoints
        self.theta = theta
        points_seq = self.waypoints
        # Only yaw angle required (no ratotions around x and y axes) in deg:
        yaweulerangles_seq = self.theta
        #List of goal quaternions:
        quat_seq = list()
        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        #rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        j = 1

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                    #Then reset movebase client :) 
                    self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                    rospy.loginfo("Waiting for move_base action server...")
                    wait = self.client.wait_for_server(rospy.Duration(5.0))
                    if not wait:
                        rospy.logerr("Action server not available!")
                        rospy.signal_shutdown("Action server not available!")
                        return
                    rospy.loginfo("Connected to move base server")
                    rospy.loginfo("Starting goals achievements ...")
                    
                    next_goal = MoveBaseGoal()
                    next_goal.target_pose.header.frame_id = "map"
                    next_goal.target_pose.header.stamp = rospy.Time.now()
                    next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                    rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                    rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                    self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached! Reversing List")
                #rospy.signal_shutdown("Final goal pose reached!")
                #exit()

                #self.waypoints = self.waypoints[0:-3]
                #self.theta = self.theta[0:-1]
                #self.waypoints = self.waypoints.reverse()
                #self.theta = self.theta.reverse()
                
                
                self.__init__(waypoints,theta)
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        
        df = pd.read_csv('~/ros_ws/src/leo_navigation/waypoints/waypoints_deskspace.csv', sep=',', header=None)
        theta = list(df.loc[:,3].values)
        wayp = df.loc[:,0:2]
        waypoints = []
        wayp = wayp.values.tolist()
        for sublist in wayp:
          for item in sublist:
             waypoints.append(item)

        MoveBaseSeq(waypoints,theta)
        rospy.loginfo("DONE DONE DONE DONE DONE DONE DONE DONE DONE DONE DONE DONE DONE DONEDONDEONDEONDE")
        
        theta.reverse()
        waypoints.reverse()
        MoveBaseSeq(waypoints,theta)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
