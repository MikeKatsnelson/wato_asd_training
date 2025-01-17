#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
//#include "nav_msgs/OccupancyGrid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
//#include "sensor_msgs/LaserScan.hpp"
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    //void publishMessage();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid,int y_grid);
    void inflateObstacles();
    void publishCostmap();

  private:
    robot::CostmapCore costmap_;
    std::vector<std::vector<int>> costmapVector;
    float width;
    float height;
    float resolution;
    uint arrayHeight;
    uint arrayWidth;


    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};
 
#endif 
/*
void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    initializeCostmap();
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 
    // Step 4: Publish costmap
    publishCostmap();
}

*/

