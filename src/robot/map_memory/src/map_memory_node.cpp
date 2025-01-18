#include  <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.h>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : 
    Node("map_memory"), 
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    last_x(0.0),
    last_y(0.0),
    distance_threshold(5.0)
  {
  // init subscribers to /costmap and /odom_filtered
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // init publisher to /map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // init timer to call updateMap every 1s
  timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

/*
 * Get a "local costmap" based on lidar scan
 */
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // store as latest costmap
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

/*
 * Get "odometry" / positioning of robot
 */
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // robot's current coordinates
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // compute distance traveled from last map update
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  
  // set new robot position if robot traveled great enough distance
  if (distance >= distance_threshold) {
      last_x = x;
      last_y = y;
      should_update_map_ = true;

      latest_odom_ = *msg;

      RCLCPP_INFO(this->get_logger(), "distance threshold passed");
  }
}

/*
 * Check every second whether global costmap should be updated
 * Update global costmap if conditions are met
 */
void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    // integrate latest "local" costmap into global costmap
    integrateCostmap();
    
    // publish
    map_pub_->publish(global_map_);
    should_update_map_ = false;

    RCLCPP_INFO(this->get_logger(), "Publishing: New global map");
  }
}

  /*
   *  Integrate latest "local" costmap into global costmap
   */
  void MapMemoryNode::integrateCostmap() {
    // robot's current position and orientation
    int robot_x = latest_odom_.pose.pose.position.x;
    int robot_y = latest_odom_.pose.pose.position.y;
    double robot_yaw = quaternionToYaw(latest_odom_.pose.pose.orientation);

    for (int j = 0; j < latest_costmap_.info.height; j++) {
      for (int i = 0; i < latest_costmap_.info.width; i++) {
        // index in local costmap
        int local_index = j * latest_costmap_.info.width + i;

        // cell is unknown -> do not update global map
        if (latest_costmap_.data[local_index] == -1)
          continue;

        // "local" coordinates of cell
        int local_x = latest_costmap_.info.origin.position.x + i;
        int local_y = latest_costmap_.info.origin.position.y + j;

        // get the "global" coordinates of the cell
          // i.e where it actually is on the global map
        int global_x = local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw) + robot_x;
        int global_y = local_x * std::sin(robot_yaw) - local_y * std::cos(robot_yaw) + robot_y;
        
        // translate global cooridnates into "cells" in costmap
        int global_i = global_x - global_map_.info.origin.position.x;
        int global_j = global_y - global_map_.info.origin.position.y;

        // skip if index is out of bounds
        if ((global_i < 0 || global_i >= global_map_.info.width) || (global_j < 0 || global_j >= global_map_.info.height)) 
          continue;

        // index in global costmap
        int global_index = global_j * global_map_.info.width + global_i;

        // overwrite old value in global costmap with new one
        global_map_.data[global_index] = latest_costmap_.data[local_index];
      }
    }
  }

  /*
   * Helper function
   */
  double MapMemoryNode::quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
  }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
