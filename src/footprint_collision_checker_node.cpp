// footprint_collision_checker_node.cpp
// Place this in: leo_navigation/src/footprint_collision_checker_node.cpp

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "leo_navigation/CheckPoseCollision.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>

class FootprintCollisionChecker
{
public:
    FootprintCollisionChecker() : 
        tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // Get parameters
        private_nh.param<std::string>("global_frame", global_frame_, "map");
        private_nh.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        
        // Initialize the costmap subscriber
        // This will use the same costmap as move_base
        ROS_INFO("Initializing Footprint Collision Checker...");
        ROS_INFO("Waiting for costmap to be available...");
        
        try {
            // Subscribe to the existing move_base global costmap
            // The costmap topic is published by move_base
            costmap_sub_ = nh.subscribe("/move_base/global_costmap/costmap", 1, 
                                       &FootprintCollisionChecker::costmapCallback, this);
            
            costmap_updates_sub_ = nh.subscribe("/move_base/global_costmap/costmap_updates", 1,
                                                &FootprintCollisionChecker::costmapUpdateCallback, this);
            
            // Wait for the first costmap message
            ros::Duration timeout(10.0);
            ros::Time start_time = ros::Time::now();
            while (!costmap_received_ && ros::ok()) {
                ros::spinOnce();
                if (ros::Time::now() - start_time > timeout) {
                    ROS_ERROR("Timeout waiting for costmap. Is move_base running?");
                    ros::shutdown();
                    return;
                }
                ros::Duration(0.1).sleep();
            }
            
            ROS_INFO("Costmap received successfully");
            
            // Get the footprint from the parameter server
            // This reads from your costmap_common.yaml
            loadFootprint();
            
        }
        catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize: %s", e.what());
            ros::shutdown();
            return;
        }
        
        // Advertise the service
        service_ = nh.advertiseService("check_pose_collision", 
                                      &FootprintCollisionChecker::checkPoseCallback, 
                                      this);
        
        ROS_INFO("Footprint Collision Checker Service is ready!");
        ROS_INFO("Service available at: %s", service_.getService().c_str());
        ROS_INFO("Using footprint with %zu points", footprint_.size());
    }
    
    ~FootprintCollisionChecker()
    {
    }
    
    /**
     * @brief Load footprint from parameter server
     */
    void loadFootprint()
    {
        ros::NodeHandle nh;
        
        // Try to get footprint from move_base's global_costmap namespace
        XmlRpc::XmlRpcValue footprint_xmlrpc;
        std::string footprint_param;
        
        // Check multiple possible locations for the footprint parameter
        if (nh.getParam("/move_base/global_costmap/footprint", footprint_xmlrpc)) {
            footprint_param = "/move_base/global_costmap/footprint";
        }
        else if (nh.getParam("/move_base/local_costmap/footprint", footprint_xmlrpc)) {
            footprint_param = "/move_base/local_costmap/footprint";
        }
        else {
            ROS_ERROR("Could not find footprint parameter! Using default rectangular footprint.");
            useDefaultFootprint();
            return;
        }
        
        // Parse the footprint - handle both array and string formats
        if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            // Standard array format: [[x1,y1],[x2,y2],...]
            for (int i = 0; i < footprint_xmlrpc.size(); ++i) {
                if (footprint_xmlrpc[i].getType() == XmlRpc::XmlRpcValue::TypeArray &&
                    footprint_xmlrpc[i].size() == 2) {
                    
                    geometry_msgs::Point pt;
                    pt.x = static_cast<double>(footprint_xmlrpc[i][0]);
                    pt.y = static_cast<double>(footprint_xmlrpc[i][1]);
                    pt.z = 0.0;
                    footprint_.push_back(pt);
                }
            }
        }
        else if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString) {
            // String format: "[[x1,y1],[x2,y2],...]"
            std::string footprint_string = static_cast<std::string>(footprint_xmlrpc);
            ROS_INFO("Footprint stored as string, parsing: %s", footprint_string.c_str());
            
            if (!parseFootprintString(footprint_string)) {
                ROS_ERROR("Failed to parse footprint string! Using default footprint.");
                useDefaultFootprint();
                return;
            }
        }
        else {
            ROS_ERROR("Footprint parameter has unexpected type! Using default footprint.");
            useDefaultFootprint();
            return;
        }
        
        if (footprint_.empty()) {
            ROS_ERROR("Parsed footprint is empty! Using default footprint.");
            useDefaultFootprint();
            return;
        }
        
        ROS_INFO("Loaded footprint from %s with %zu points", 
                 footprint_param.c_str(), footprint_.size());
    }
    
    /**
     * @brief Parse footprint from string format "[[x1,y1],[x2,y2],...]"
     */
    bool parseFootprintString(const std::string& footprint_string)
    {
        // Remove all whitespace
        std::string str = footprint_string;
        str.erase(std::remove_if(str.begin(), str.end(), ::isspace), str.end());
        
        // Check for outer brackets
        if (str.size() < 4 || str[0] != '[' || str[str.size()-1] != ']') {
            return false;
        }
        
        // Remove outer brackets
        str = str.substr(1, str.size() - 2);
        
        // Parse each [x,y] pair
        size_t pos = 0;
        while (pos < str.size()) {
            // Find opening bracket
            if (str[pos] != '[') {
                return false;
            }
            pos++; // Skip '['
            
            // Find closing bracket
            size_t end = str.find(']', pos);
            if (end == std::string::npos) {
                return false;
            }
            
            // Extract the pair string "x,y"
            std::string pair = str.substr(pos, end - pos);
            
            // Find comma
            size_t comma = pair.find(',');
            if (comma == std::string::npos) {
                return false;
            }
            
            // Parse x and y
            try {
                double x = std::stod(pair.substr(0, comma));
                double y = std::stod(pair.substr(comma + 1));
                
                geometry_msgs::Point pt;
                pt.x = x;
                pt.y = y;
                pt.z = 0.0;
                footprint_.push_back(pt);
            }
            catch (const std::exception& e) {
                ROS_ERROR("Failed to parse footprint coordinates: %s", e.what());
                return false;
            }
            
            pos = end + 1; // Move past ']'
            
            // Skip comma between pairs
            if (pos < str.size() && str[pos] == ',') {
                pos++;
            }
        }
        
        return !footprint_.empty();
    }
    
    /**
     * @brief Use default hardcoded footprint
     */
    void useDefaultFootprint()
    {
        footprint_.clear();
        geometry_msgs::Point pt;
        pt.z = 0.0;
        
        pt.x = 0.21; pt.y = 0.22; footprint_.push_back(pt);
        pt.x = 0.21; pt.y = -0.22; footprint_.push_back(pt);
        pt.x = -0.21; pt.y = -0.22; footprint_.push_back(pt);
        pt.x = -0.21; pt.y = 0.22; footprint_.push_back(pt);
        
        ROS_INFO("Using default hardcoded footprint with %zu points", footprint_.size());
    }
    
    /**
     * @brief Callback for costmap messages
     */
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(costmap_mutex_);
        
        // Store the costmap information
        costmap_info_.resolution = msg->info.resolution;
        costmap_info_.width = msg->info.width;
        costmap_info_.height = msg->info.height;
        costmap_info_.origin_x = msg->info.origin.position.x;
        costmap_info_.origin_y = msg->info.origin.position.y;
        
        // Copy the data
        costmap_data_ = msg->data;
        costmap_received_ = true;
        
        ROS_INFO_ONCE("Costmap data received: %dx%d, resolution: %.3f", 
                     costmap_info_.width, costmap_info_.height, costmap_info_.resolution);
    }
    
    /**
     * @brief Callback for costmap update messages
     */
    void costmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(costmap_mutex_);
        
        if (!costmap_received_) return;
        
        // Apply updates to the costmap
        for (size_t i = 0; i < msg->data.size(); ++i) {
            int x = msg->x + (i % msg->width);
            int y = msg->y + (i / msg->width);
            int index = y * costmap_info_.width + x;
            
            if (index >= 0 && index < costmap_data_.size()) {
                costmap_data_[index] = msg->data[i];
            }
        }
    }
    
    /**
     * @brief Service callback to check pose collision
     */
    bool checkPoseCallback(leo_navigation::CheckPoseCollision::Request& req,
                          leo_navigation::CheckPoseCollision::Response& res)
    {
        if (!costmap_received_) {
            ROS_WARN("Costmap not yet received, cannot check collision");
            res.is_free = false;
            res.cost = -1.0;
            res.message = "Costmap not available";
            return true;
        }
        
        // Transform pose to map frame if needed
        geometry_msgs::PoseStamped pose_in_map;
        try {
            if (req.pose.header.frame_id != global_frame_) {
                tf_buffer_.transform(req.pose, pose_in_map, global_frame_, 
                                    ros::Duration(1.0));
            } else {
                pose_in_map = req.pose;
            }
        }
        catch (tf2::TransformException& ex) {
            ROS_ERROR("Transform failed: %s", ex.what());
            res.is_free = false;
            res.cost = -1.0;
            res.message = "Transform failed: " + std::string(ex.what());
            return true;
        }
        
        // Transform footprint to the query pose
        std::vector<geometry_msgs::Point> oriented_footprint = 
            transformFootprint(pose_in_map.pose, footprint_);
        
        // Check collision
        double max_cost = checkFootprintCost(oriented_footprint);
        
        // Interpret the cost
        // OccupancyGrid values: 0 = free, 100 = occupied, -1 = unknown
        // We need to convert these to costmap_2d values
        bool is_free = true;
        std::string message;
        
        if (max_cost < 0) {
            is_free = false;
            message = "Footprint is outside map bounds or in unknown space";
        }
        else if (max_cost >= 99) {
            is_free = false;
            message = "Footprint collides with lethal obstacle";
        }
        else if (max_cost >= 50) {
            is_free = false;
            message = "Footprint is too close to obstacles";
        }
        else {
            is_free = true;
            message = "Pose is collision-free";
        }
        
        res.is_free = is_free;
        res.cost = max_cost;
        res.message = message;
        
        // ROS_INFO("Checked pose (%.2f, %.2f): %s (cost: %.2f)", 
        //          pose_in_map.pose.position.x, 
        //          pose_in_map.pose.position.y,
        //          is_free ? "FREE" : "COLLISION",
        //          max_cost);
        
        return true;
    }
    
    /**
     * @brief Check the cost of a footprint on the costmap
     */
    double checkFootprintCost(const std::vector<geometry_msgs::Point>& footprint)
    {
        boost::mutex::scoped_lock lock(costmap_mutex_);
        
        if (footprint.empty() || costmap_data_.empty()) {
            return -1.0;
        }
        
        double max_cost = 0.0;
        
        // Find the bounding box of the footprint
        double min_x = footprint[0].x;
        double max_x = footprint[0].x;
        double min_y = footprint[0].y;
        double max_y = footprint[0].y;
        
        for (const auto& pt : footprint) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }
        
        // Convert world coordinates to grid coordinates
        int min_grid_x = worldToGridX(min_x);
        int max_grid_x = worldToGridX(max_x);
        int min_grid_y = worldToGridY(min_y);
        int max_grid_y = worldToGridY(max_y);
        
        // Check all cells in the bounding box
        for (int grid_y = min_grid_y; grid_y <= max_grid_y; ++grid_y) {
            for (int grid_x = min_grid_x; grid_x <= max_grid_x; ++grid_x) {
                // Check if this cell is inside the grid
                if (grid_x < 0 || grid_x >= costmap_info_.width ||
                    grid_y < 0 || grid_y >= costmap_info_.height) {
                    return -1.0; // Outside bounds
                }
                
                // Convert grid coordinates back to world
                double world_x = gridToWorldX(grid_x);
                double world_y = gridToWorldY(grid_y);
                
                // Check if this point is inside the footprint polygon
                if (pointInPolygon(world_x, world_y, footprint)) {
                    int index = grid_y * costmap_info_.width + grid_x;
                    int8_t cost = costmap_data_[index];
                    
                    // Handle unknown space (-1) as obstacle
                    if (cost == -1) {
                        return -1.0;
                    }
                    
                    max_cost = std::max(max_cost, static_cast<double>(cost));
                    
                    // Early exit if we hit a lethal obstacle
                    if (max_cost >= 99) {
                        return max_cost;
                    }
                }
            }
        }
        
        return max_cost;
    }
    
    /**
     * @brief Check if a point is inside a polygon using ray casting
     */
    bool pointInPolygon(double x, double y, const std::vector<geometry_msgs::Point>& polygon)
    {
        int num_vertices = polygon.size();
        bool inside = false;
        
        for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
            double xi = polygon[i].x, yi = polygon[i].y;
            double xj = polygon[j].x, yj = polygon[j].y;
            
            bool intersect = ((yi > y) != (yj > y)) &&
                           (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        
        return inside;
    }
    
    /**
     * @brief Transform footprint to a specific pose
     */
    std::vector<geometry_msgs::Point> transformFootprint(
        const geometry_msgs::Pose& pose,
        const std::vector<geometry_msgs::Point>& footprint)
    {
        std::vector<geometry_msgs::Point> transformed_footprint;
        
        // Extract yaw from quaternion
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double cos_th = cos(yaw);
        double sin_th = sin(yaw);
        
        for (const auto& pt : footprint) {
            geometry_msgs::Point new_pt;
            new_pt.x = pose.position.x + (pt.x * cos_th - pt.y * sin_th);
            new_pt.y = pose.position.y + (pt.x * sin_th + pt.y * cos_th);
            new_pt.z = 0.0;
            transformed_footprint.push_back(new_pt);
        }
        
        return transformed_footprint;
    }
    
    // Coordinate conversion helpers
    int worldToGridX(double wx) {
        return static_cast<int>((wx - costmap_info_.origin_x) / costmap_info_.resolution);
    }
    
    int worldToGridY(double wy) {
        return static_cast<int>((wy - costmap_info_.origin_y) / costmap_info_.resolution);
    }
    
    double gridToWorldX(int gx) {
        return costmap_info_.origin_x + (gx + 0.5) * costmap_info_.resolution;
    }
    
    double gridToWorldY(int gy) {
        return costmap_info_.origin_y + (gy + 0.5) * costmap_info_.resolution;
    }
    
private:
    // ROS components
    ros::ServiceServer service_;
    ros::Subscriber costmap_sub_;
    ros::Subscriber costmap_updates_sub_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Costmap data
    struct CostmapInfo {
        double resolution;
        int width;
        int height;
        double origin_x;
        double origin_y;
    } costmap_info_;
    
    std::vector<int8_t> costmap_data_;
    boost::mutex costmap_mutex_;
    bool costmap_received_ = false;
    
    // Robot footprint
    std::vector<geometry_msgs::Point> footprint_;
    
    // Parameters
    std::string global_frame_;
    std::string robot_base_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "footprint_collision_checker");
    
    FootprintCollisionChecker checker;
    
    ros::spin();
    
    return 0;
}