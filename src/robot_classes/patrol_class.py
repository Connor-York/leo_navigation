#!/usr/bin/env python3
import rospy


class Patroller:
    
    def __init__(self, package_path):
        
        # Initialise Patrolling 
        patrol_route = package_path + "/waypoints/" + rospy.get_param("~patrol_route")
        self.waypoint_list = self.waypoint_gen(patrol_route)
        self.current_goal = rospy.get_param("~start_node")
        self.current_lap = 0
        
        
        
        
    def waypoint_gen(self, waypoint_csv):
        # preprocessing --------------------------------------------------
        # converts waypoints text file into a list of points to follow
        df = pd.read_csv(waypoint_csv, sep=',', header=None)
        theta = list(df.loc[:, 3].values)
        wayp = df.loc[:, 0:2]
        waypoints = []
        wayp = wayp.values.tolist()
        for sublist in wayp:
            for item in sublist:
                waypoints.append(item)

        points_seq = waypoints  # coordinates for each waypoint
        yaweulerangles_seq = theta  # heading angle for each waypoint

        # Convert waypoint & heading values into a list of robot poses (quaternions?) -----------
        quat_seq = list()
        # List of goal poses:
        pose_seq = list()
        for yawangle in yaweulerangles_seq:
            # Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(
                *(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            # Exploit n variable to cycle in quat_seq
            pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
            n += 1
            
        return pose_seq