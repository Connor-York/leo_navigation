#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
import datetime
import rospkg 
import csv
import os
import time


startTime = time.perf_counter()

# def rosInit():
    

    # rospy.on_shutdown(saveCSV)

def getTime():

    #grabbing time and date to provide unique ID for logs
    dateTime = datetime.now()
    dtString = dateTime.strftime("%Y%m%d%H%M%S") #ISO 8601 Standard

    rosTimeUnf = rospy.Time.now()

    rosCurrentTime = datetime.fromtimestamp(rosTimeUnf.to_sec())

    rosTime = rosCurrentTime.strftime("%H:%M: %S")

    return dtString, rosTime


def getPath():

    timenow, _ = getTime()


    rp = rospkg.RosPack()
    packagePath = rp.get_path('arLogger')

    logFolder = os.path.join(packagePath, "logs")


    # velPath = os.path.join(logFolder, folderName + "vel")
    # posePath = os.path.join(logFolder,folderName + "pose")
    # batPath = os.path.join(logFolder,folderName + "bat")


    velFullpath = os.path.join(logFolder, timenow + "_vellog.csv")
    poseFullpath = os.path.join(logFolder, timenow + "_poselog.csv")
    batFullpath = os.path.join(logFolder, timenow + "_batlog.csv")

    print (velFullpath)
    print(poseFullpath)
    print(batFullpath)

    return logFolder, velFullpath, poseFullpath, batFullpath

# def makeFolder():

#     path, _ , _ , _= getPath()

#     testFile = None

#     # test folder permisions
#     try:
#         testFile = open(os.path.join(path, 'test.txt'), 'w+')
#     except IOError:
#         try:
#             os.mkdir(path)
#         except OSError:
#             print("No log folder created")
#         else:
#             print("Log folder created")

#     testFile.close()
#     os.remove(testFile.name)



def velCallback(msg):

    global startTime

    finishTime = time.perf_counter()

    timeStamp = round(finishTime-startTime, 2)


    _, velPath, _, _ = getPath()


    lin_x = msg.linear.x
    ang_z = msg.angular.z
    _, timeNow = getTime()
    
    velData = [timeNow, timeStamp, lin_x, ang_z]

    velHeaders =" 'Ros Time', 'Time Stamp', 'Linear Velocity', 'Angular Velocity' "

    saveCSV(velPath, velData, velHeaders)
    

def poseCallback(msg):

    global startTime

    finishTime = time.perf_counter()

    timeStamp = round(finishTime-startTime, 2)


    _, _, posePath, _ = getPath()

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w

    _, timeNow = getTime()

    poseData = [timeNow, timeStamp, x, y, ox, oy, oz, ow]

    poseHeaders = " 'Ros Time', 'Time Stamp', 'x', 'y', 'ox', 'oy', 'oz', 'ow' "

    saveCSV(posePath, poseData, poseHeaders)


def batCallback(msg):

    global startTime

    finishTime = time.perf_counter()

    timeStamp = round(finishTime-startTime, 2)

    _ , _, _, batPath = getPath()

    _, timeNow = getTime()

    batData = [timeNow, timeStamp, msg.data]

    batHeaders = " 'Ros Time', 'Time Stamp', 'Battery Level' "

    saveCSV(batPath, batData, batHeaders)


def saveCSV(filename, data, headers):

    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([headers])
        writer.writerow([data])

if __name__ == "__main__":
    
    rospy.init_node("jogger")

    vel_subscriber = rospy.Subscriber("nav_vel", Twist, velCallback)
    pose_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, poseCallback)
    battery_subscriber = rospy.Subscriber("firmware/battery_averaged", Float32, batCallback)


    # makeFolder()

    rospy.spin()