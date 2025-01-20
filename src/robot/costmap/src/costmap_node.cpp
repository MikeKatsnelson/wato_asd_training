#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostmap, this));
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar",1,std::bind(&CostmapNode::laserCallback,this,std::placeholders::_1));//queue size of 1, _1 indicates the first argument in this case scan 
}

void CostmapNode::initializeCostmap(){
  costmapVector.resize(arrayHeight,std::vector<int>(arrayWidth,0));
   for (int i = 0; i<arrayHeight;i++){
    for (int j = 0; j<arrayWidth;j++){
      costmapVector[i][j] = 0;
    }
   }
}

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid){//this acts as if we were in a corner
  x_grid = range*std::cos(angle);
  y_grid = range*std::sin(angle);
}

//void CostmapNode::markObstacle(int x_grid,int y_grid){
//  costmapVector[y_grid][x_grid] = 100;
//}

void CostmapNode::inflateObstacles(){
  double inflation_rad = 1.0/resolution;
  double max_cost = 100.0;
  for (int i = 0; i<arrayHeight;i++){
    for (int j = 0; j<arrayWidth;j++){
      if (costmapVector[i][j] == 100){
        //for every obstacle cell
        for (int k = 0; k<arrayHeight;k++)
          {
            for (int l = 0; l<arrayWidth; l++){
              double distance = std::sqrt( std::pow(i-k,2) + std::pow(j-l,2));
              int temp_cost = max_cost*(1-(distance/inflation_rad));
              if (temp_cost > costmapVector[k][l]){
                costmapVector[k][l] = temp_cost;
              }
            }
          }
      } 
    }
  }

}


// Define the timer to publish a message every 500ms
void CostmapNode::publishCostmap() {
  double index=0;

  std::vector<int8_t> OccupancyMap(arrayHeight*arrayWidth);//int[arrayHeight*arrayWidth] OccupancyMap;
  for (int i = 0; i<arrayHeight;i++){
    for (int j =0; j<arrayWidth;j++){
      OccupancyMap[index] = costmapVector[i][j];
      index++;
 }
 }
  auto message = nav_msgs::msg::OccupancyGrid();
  //message.data = "Hello, ROS 2!";
  message.header.stamp = this->get_clock()->now();
  message.info.resolution =resolution;
  message.info.width = arrayWidth;
  message.info.height = arrayHeight;
  message.info.origin.position.x = -width/2;
  message.info.origin.position.y = -height/2;

  message.header.frame_id = "robot/chassis/lidar";//"/costmap";//"Costmap Occupancy Grid";
  //message.header = msg->header;
  //message.info.origin  = ;
  message.data = OccupancyMap;
  //RCLCPP_INFO(this->get_logger(), "Publishing: Occupancy Grid");//, message.data.c_str());
  costmap_pub_->publish(message);
}




void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
// Step 1: Initialize costmap
    width = 40;
    height = 40;
    resolution = 0.1;//0.1;
    arrayHeight = height/resolution;
    arrayWidth = width/resolution;
   
    initializeCostmap();


    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range/resolution, angle, x_grid, y_grid);

            x_grid += arrayWidth/2;
            y_grid += arrayHeight/2;
            if (x_grid < arrayWidth && x_grid >=0 && y_grid <arrayHeight && y_grid >=0){
              
              costmapVector[y_grid][x_grid] = 100;//markObstacle(x_grid, y_grid);
            }
          
            
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 



    // Step 4: Publish costmap
    publishCostmap();
}


 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}