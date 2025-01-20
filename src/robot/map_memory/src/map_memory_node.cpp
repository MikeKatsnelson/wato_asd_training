#include  <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.h>

#include <set>
#include <utility>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : 
    Node("map_memory"), 
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    last_x(0.0),
    last_y(0.0),
    distance_threshold(2)
  {
  // init subscribers to /costmap and /odom_filtered
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // init publisher to /map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // init timer to call updateMap every 1s
  // timer_ = this->create_wall_timer(
  //           std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  timer_ = this->create_wall_timer(
             std::chrono::seconds(1), std::bind(&MapMemoryNode::publishMap, this));

  // init global map
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = 0.3;
  global_map_.info.width = std::ceil(30 / global_map_.info.resolution);
  global_map_.info.height = std::ceil(30 / global_map_.info.resolution);
  global_map_.info.origin.position.x = -15;
  global_map_.info.origin.position.y = -15;
  // global_map_.info.origin.position.x =  -(static_cast<double>(global_map_.info.width) / 2.0);
  // global_map_.info.origin.position.y =-(static_cast<double>(global_map_.info.height) / 2.0);

  global_map_.data = {};
  global_map_.data.resize(global_map_.info.width * global_map_.info.height);
  std::fill(global_map_.data.begin(), global_map_.data.end(), 0);
}

/*
 * Get a "local costmap" based on lidar scan
 */
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;

  bool not_empty_costmap = false;
  for (int i = 0; i < msg->data.size(); ++i)
  {
    if (msg->data[i] != 0)
    {
        not_empty_costmap = true;
        break;
    }
  }

  if (should_update_map_ && not_empty_costmap) {
    updateMap();
  }
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
  }
  //latest_odom_ = *msg;
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
    //global_map_.header.stamp = this->get_clock()->now();
    //map_pub_->publish(global_map_);
    should_update_map_ = false;
    costmap_updated_ = false;

    RCLCPP_INFO(this->get_logger(), "Publishing: New global map");
  }
}

void MapMemoryNode::publishMap() {
  global_map_.header.stamp = this->get_clock()->now();

  global_map_ *= global_map_.info.resolution;
  map_pub_->publish(global_map_);
  global_map_ /= global_map_.info.resolution;
}

/*
 *  Integrate latest "local" costmap into global costmap
*/
void MapMemoryNode::integrateCostmap() {
  // robot's coordinates & rotation
  int robot_x = latest_odom_.pose.pose.position.x;
  int robot_y = latest_odom_.pose.pose.position.y;
  double robot_yaw = quaternionToYaw(latest_odom_.pose.pose.orientation);

  /* -- FOR DEBUGGING -- */
  // RCLCPP_INFO(this->get_logger(), "x: %d, y: %d", robot_x, robot_y);
  // RCLCPP_INFO(this->get_logger(), "yaw: %f", robot_yaw);
  // int local_robot_x = 0;
  // int local_robot_y = 0;
  // int global_robot_x = local_robot_x * std::cos(robot_yaw) - local_robot_y * std::sin(robot_yaw) + robot_x;
  // int global_robot_y = local_robot_x * std::sin(robot_yaw) + local_robot_y * std::cos(robot_yaw) + robot_y;
  // RCLCPP_INFO(this->get_logger(), "global_robot_x: %d, global_robot_y: %d", global_robot_x, global_robot_y);
  /* -- -- */

  for (int j = 0; j < latest_costmap_.info.height; j++) {
    //std::set<std::pair<int, int>> global_indices;

    for (int i = 0; i < latest_costmap_.info.width; i++) {
      // index in local costmap
      int local_index = j * latest_costmap_.info.width + i;

      // cell is unknown OR 0 -> do not update global map
      if (latest_costmap_.data[local_index] == -1 || latest_costmap_.data[local_index] == 0) {
        continue;
      }

      // "local" coordinates of cell
      double local_x = (i * latest_costmap_.info.resolution) + latest_costmap_.info.origin.position.x;
      double local_y = (j * latest_costmap_.info.resolution) + latest_costmap_.info.origin.position.y;
      
     //if (i == latest_costmap_.info.width - 1)
        //RCLCPP_INFO(this->get_logger(), "local_x: %f, local_y: %f", local_x, local_y);     

      // get the "global" coordinates of the cell
        // i.e where it actually is on the global map
      double global_x = local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw) + robot_x;
      double global_y = local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw) + robot_y;
      
      //if (i = latest_costmap_.info.width - 1)
        //RCLCPP_INFO(this->get_logger(), "global_x: %f, global_y: %f", global_x, global_y);      
      
      // translate global cooridnates into "cells" in costmap
      int global_i = (global_x - global_map_.info.origin.position.x) / global_map_.info.resolution;
      int global_j = (global_y - global_map_.info.origin.position.y) / global_map_.info.resolution;
      
      // if (i == latest_costmap_.info.width - 1) {
      //   //RCLCPP_INFO(this->get_logger(), "resoluton: %f, origin.x: %f, origin.y: %f", global_map_.info.resolution, global_map_.info.origin.position.x, global_map_.info.origin.position.y);      
      //   // RCLCPP_INFO(this->get_logger(), "global_i: %d, global_j: %d", global_i, global_j);  
      // }    

      // std::pair global_index_pair = {global_x, global_y};
      // if (global_indices.find(global_index_pair) != global_indices.end()) {
      //   continue;
      // }
      // global_indices.insert(global_index_pair);

      // skip if index is out of bounds
      
      if ((global_i < 0 || global_i >= global_map_.info.width) || (global_j < 0 || global_j >= global_map_.info.height)) {
        continue;
      }

      // index in global costmap
      int global_index = global_j * global_map_.info.width + global_i;

      RCLCPP_INFO(this->get_logger(), "-- Valid Index --");
      RCLCPP_INFO(this->get_logger(), "local_i: %d, local_j: %d", i, j);
      RCLCPP_INFO(this->get_logger(), "local_x: %f, local_y: %f", local_x, local_y);
      RCLCPP_INFO(this->get_logger(), "global_x: %f, global_y: %f", global_x, global_y);
      RCLCPP_INFO(this->get_logger(), "global_i: %d, global_j: %d", global_i, global_j);
      RCLCPP_INFO(this->get_logger(), "yaw: %f", robot_yaw);

      // if (latest_costmap_.data[local_index] > 0)
      //   RCLCPP_INFO(this->get_logger(), "costmap data: %d, global map data: %d", latest_costmap_.data[local_index], global_map_.data[global_index]);

      RCLCPP_INFO(this->get_logger(), "costmap data: %d, global map data: %d", latest_costmap_.data[local_index], global_map_.data[global_index]);

      // only overwrite global costmap if an object was detected in the same cell in local costmap
        // do this by taking the max of equilavent cells in local and global costmaps
      global_map_.data[global_index] = std::max(latest_costmap_.data[local_index], global_map_.data[global_index]);

      // overwrite old value in global costmap with new one
      //global_map_.data[global_index] = latest_costmap_.data[local_index];
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

  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
