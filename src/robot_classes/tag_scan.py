import rospy
import time
from ar_track_alvar_msgs.msg import AlvarMarkers
import csv

"""
TO-DO:
check reward_id_seen/ not_seen and reintegrate away from globals
check imports
"""

class Tag_scan(): #==============================================================================

    def __init__(self,start_time,reward_csv_path):
        self.ID_list = []
        self.buffer = []
        self.rewards = rospy.get_param("~rewards") #list of tags which give reward
        rewards_2 = rospy.get_param("~rewards_2") #second list to switch to after X time
        self.LI = rospy.get_param("~latent_inhibition") #0 = always rescan 1 = never rescan (floating val)

        self.num_tags = rospy.get_param("~num_tags") #number of tags to read at each waypoint

        self.start_time = start_time
        self.reward_csv_path = reward_csv_path

        current_time = time.time() - self.start_time

        time_switch = rospy.get_param("~time_switch") * 60 #Time after which to switch rewarding tags
        
        dynamic_env = rospy.get_param("~dynamic_env") #whether to switch tag rewards

        if current_time >= time_switch and dynamic_env == True:
            print("Environment switching ==================================================================")
            # print("Old rewards: ")
            # print(self.rewards)
            self.rewards = rewards_2
            # print("New rewards: ")
            # print(self.rewards) 

        self.ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.check_ID_callback)
        
        self.complete_scan = 0
        self.callback_tick = 1
        rospy.loginfo('finished init func')

    def tag_scan(self):

        #rospy.loginfo("TAG SCAN") 

        if self.callback_tick == 0:
            scanned_prev = reward_ID_seen + no_reward_ID_seen
            print(scanned_prev)
            print(reward_ID_seen)
            print(no_reward_ID_seen)
            rospy.loginfo(self.ID_list)
            for ID in self.ID_list: 
                # this logic is definitely flawed, test and fix to not
                # put tags in both? Or to handle the switching case.
                if ID not in scanned_prev: # if new, scan for the first time
                    rospy.loginfo("Scanning new ID - " + str(ID))
                    self.scan(ID)
                elif ID in reward_ID_seen: # if known reward, scan
                    rospy.loginfo("Scanning known reward ID - " + str(ID))
                    self.scan(ID)
                elif ID in no_reward_ID_seen: # only re-scan known no reward if chance 
                    chance = 1 - self.LI # High LI is low chance, LOW LI is high chance :) 
                    r = random.random() #float in range 0-1
                    print("CHANCE")
                    print(chance)
                    print(r)
                    if r <= chance: 
                        rospy.loginfo("Rescanning known no reward ID - " + str(ID))
                        self.scan(ID) #rescan :) 
                    else:
                        rospy.loginfo("Ignoring known no reward ID - " + str(ID))
            rospy.loginfo("scan forloop complete, complete_scan = 1")
            self.complete_scan = 1
            rospy.loginfo("Reward Count: " + str(reward_count))
        #rospy.loginfo("After for loop")
        
        
        rospy.sleep(1) #If i remove this it leaves tag scan too early I think.
        if self.complete_scan == 0:
            self.tag_scan()
        self.ar_subscriber.unregister() 
        rospy.loginfo("unsubscribed")

    def scan(self,ID): # GLOBALS EDITED HERE
        global reward_ID_seen
        global no_reward_ID_seen
        global reward_count
        self.scan_delay() # delay to simulate doing something lmao
        if ID in self.rewards:
            if ID not in reward_ID_seen:
                reward_ID_seen.append(ID)
            if ID in no_reward_ID_seen:
                no_reward_ID_seen.remove(ID)
            rospy.loginfo("Reward got!")
            reward_count += 1
            current_time = time.time() - self.start_time
            data = [reward_count,current_time]
            self.save_to_csv(self.reward_csv_path,data)
            
        else:
            if ID not in no_reward_ID_seen:
                no_reward_ID_seen.append(ID)
            if ID in reward_ID_seen:
                reward_ID_seen.remove(ID)
            rospy.loginfo("No Reward :(")


    def scan_delay(self):
        t = rospy.get_param("~scan_delay")
        for i in range(t):
            print("Scanning " + str(i+1) + "/" + str(t) + "...")
            rospy.sleep(1)
        rospy.loginfo("Scan complete")

    def save_to_csv(self,csv_path,data):
        with open(csv_path, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(data)

    def dupe_check(self, iterable,check):
        for x in iterable:
            if x == check:
                return True

    def buffer_check(self, check): 
        self.buffer.append(check)
        if len(self.buffer) == 11:
            self.buffer.pop(0)
        bick = 0 #buffer tick
        print("BUFFER CHECK: ")
        print(self.buffer)
        for x in self.buffer: 
            if x == check:
                bick += 1
                #print(bick)
                if bick >= 3: # if 3 of the same tag in buffer
                    return True

    def check_ID_callback(self, msg): #callback triggered by ar_tag subscriber
        if self.callback_tick == 1:
            for marker in msg.markers:
                #filter out fake IDs (>17), any IDs already in the list, and only accept those that have been seen thrice
                if self.buffer_check(marker.id):
                    if marker.id < 18:
                        if self.dupe_check(self.ID_list, marker.id) == None:
                            rospy.loginfo("CALLBACK ID")
                            #current_time = time.time()
                            #elapsed_time = current_time - start_time
                            #Time_list.append(elapsed_time)
                            self.ID_list.append(marker.id)
                            if len(self.ID_list) == self.num_tags:
                                self.callback_tick = 0