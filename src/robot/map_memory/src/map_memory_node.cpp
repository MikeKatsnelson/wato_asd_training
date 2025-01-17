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

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // store latest costmap
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // compute distance traveled from last map update
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  
  // set new robot position
  if (distance >= distance_threshold) {
      last_x = x;
      last_y = y;
      should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
  }
}

// integrate the latest costmap into the global map
  void MapMemoryNode::integrateCostmap() {
    // if function is called for the 1st time
    if (global_map_.info.width == 0 && global_map_.info.height == 0) {
      // copy latest costmap into global costmap
        // takes care of grid alignment (ensures both are same size)
      global_map_ = latest_costmap_;
      return;
    }
    
    // transform and merge latest costmap into global map
    for (int i = 0; i < latest_costmap_.info.width * latest_costmap_.info.height; i++) {
      if (latest_costmap_.data[i] != -1) {
        // cell is known = update global map
        global_map_.data[i] = latest_costmap_.data[i];
      }
    }
  }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
