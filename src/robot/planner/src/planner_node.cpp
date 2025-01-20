#include "planner_node.hpp"
#include <cmath>
#include <unordered_map>
#include <queue>
#include <vector>
#include <algorithm>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())),state_(State::WAITING_FOR_GOAL) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}
 
 bool PlannerNode::goalReached() {
        double dx = goal_.point.x - robot_pose_.position.x;
        double dy = goal_.point.y - robot_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) <= 1;//<0.5; // Threshold for reaching the goal
    }

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {//received a new map so update plan
        current_map_ = *msg;
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            planPath();
        }
    }
 void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {//wait for a new goal
        goal_ = *msg;
        goal_received_ = true;
        state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
        planPath();
    }
 


    void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {//update robot position
        robot_pose_ = msg->pose.pose;
    }
 
    void PlannerNode::timerCallback() {//checks if goal is reached and will replan path
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            if (goalReached()) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                state_ = State::WAITING_FOR_GOAL;
                goal_received_ = false;
            }
             else {
                RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
                planPath();
            }
        }
        else if (state_ == State::WAITING_FOR_GOAL){
                nav_msgs::msg::Path path;
                path.poses = {};
                path.header.stamp = this->get_clock()->now();
                path.header.frame_id = "sim_world";
                RCLCPP_WARN(this->get_logger(), "publishing empty path");
                path_pub_->publish(path);
        }

    }
 #ifndef SUPPORT_STRUCTS
#define SUPPORT_STRUCTS 
struct CellIndex
{
  int x;
  int y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex &other) const{
    return (x == other.x && y == other.y); }
 
  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y); }
};

struct CellIndexHash// Hash function for CellIndex so it can be used in std::unordered_map
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 

struct AStarNode// Structure representing a node in the A* open set
{
  CellIndex index;
  CellIndex parentIndex;
  double f_score;  // f = g + h
  AStarNode(CellIndex idx, double f, CellIndex Pidx) : index(idx), f_score(f),parentIndex(Pidx) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};
#endif

double h_score(CellIndex index1, CellIndex index2){
        double h_score = std::sqrt( pow(index1.x-index2.x,2) +  pow(index1.y-index2.y,2)         );//std::abs(index1.x-index2.x) + std::abs(index1.y-index2.y);
        return h_score;
    }
void PlannerNode::planPath() {

    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }
    nav_msgs::msg::Path path;
    RCLCPP_WARN(this->get_logger(), "Planning path");
    
    //create open set
    std::unordered_map<CellIndex,AStarNode,CellIndexHash> open;//open set of nodes
    //create closed set
    std::unordered_map<CellIndex,AStarNode,CellIndexHash> close;//closed set of nodes
    //create start and end index
    int robotPosX = std::round(robot_pose_.position.x) /current_map_.info.resolution;
    int robotPosY = std::round(robot_pose_.position.y)/current_map_.info.resolution;
    int GoalPosX = std::round(goal_.point.x)/current_map_.info.resolution ;
    int GoalPosY = std::round(goal_.point.y)/current_map_.info.resolution;
    
    int adjustedWidth =  current_map_.info.width;///current_map_.info.width/current_map_.info.resolution;
    int adjustedHeight =current_map_.info.height;//current_map_.info.height/current_map_.info.resolution;

    CellIndex start(robotPosX,robotPosY);//(goal_.point.x,robot_pose_.position.y/current_map_.info.resolution);
    RCLCPP_WARN(this->get_logger(), "start pos(x:%d, y:%d)", start.x, start.y);
    //create end node
    CellIndex end(GoalPosX,GoalPosY);//(goal_.point.x/current_map_.info.resolution,goal_.point.y/current_map_.info.resolution);
    RCLCPP_WARN(this->get_logger(), "end pos(x:%d, y:%d)", end.x, end.y); // start and end indexes are with irigin in centre and coordinates in rseolution
    //actual position -4,4 would be -40,40 in res 0.1
    //add start node to open
    AStarNode current(start,0,start);
    
    open.emplace(start,current);
    //open[start] = current;
    //while open set is not empty
    bool found = false;
//RCLCPP_WARN(this->get_logger(), "0");
while (found == false && !open.empty()){
    //RCLCPP_WARN(this->get_logger(), "1");
        //make current node the open node with lowest f_score
        bool first = true;
        //AStarNode current;
        for (const auto& [key,value] : open){//set current to node with lowest f_cost
            if (first || value.f_score <current.f_score) {
                current = value;
                first = false;
            }         
        }
        //RCLCPP_WARN(this->get_logger(), "2");
        //RCLCPP_WARN(this->get_logger(), "current pos(x:%d, y:%d)", current.index.x, current.index.y);
        //remove current from open
        open.erase(current.index);
        //add current to closed
        close.emplace(current.index,current);

        //if current is the target node, path.pushback current node until home
        if (current.index == end){
            //RCLCPP_WARN(this->get_logger(), "goal found");
            found = true;
            nav_msgs::msg::Path TempPath;
            //RCLCPP_WARN(this->get_logger(), "5");
            geometry_msgs::msg::PoseStamped TraversalPose;
            while (current.index != start){
                //RCLCPP_WARN(this->get_logger(), "6");
                TraversalPose.header.frame_id = "sim_world";
                TraversalPose.pose.position.x = current.index.x*current_map_.info.resolution;
                TraversalPose.pose.position.y = current.index.y*current_map_.info.resolution;
                TempPath.poses.push_back(TraversalPose);
                
              // RCLCPP_WARN(this->get_logger(), "retrace pos(x:%d, y:%d))", current.index.x, current.index.y);//,value.f_score);
                if (close.find(current.parentIndex)!= close.end()) { //inside the close 
                  current = close.at(current.parentIndex);//current = close[current.parentIndex];  
                }
            }
            
            std::reverse(TempPath.poses.begin(), TempPath.poses.end()); 
            path.poses = TempPath.poses;
            
        }

        //for every neighbor of current node 
        std::vector<CellIndex> neighbors = {
            {current.index.x - 1, current.index.y}, 
            {current.index.x + 1, current.index.y}, 
            {current.index.x, current.index.y - 1}, 
            {current.index.x, current.index.y + 1} 
            };


        for (const CellIndex& neighbor : neighbors){
            //RCLCPP_WARN(this->get_logger(), "Caluculating neighbors");
            //RCLCPP_WARN(this->get_logger(), "neighbor pos(x:%d, y:%d)", neighbor.x, neighbor.y);
            //is the if statement correct?
            //RCLCPP_WARN(this->get_logger(), "4");
             
            if (neighbor.x < (adjustedWidth/-2) || neighbor.y  < (adjustedHeight/-2) || neighbor.x >= adjustedWidth/2 || neighbor.y >= adjustedHeight/2 
            || close.find(neighbor) != close.end() || current_map_.data[(neighbor.y+(adjustedHeight/2))*adjustedWidth + (neighbor.x+(adjustedWidth/2))] != 0 ) {//if in close or not traversable
                //RCLCPP_WARN(this->get_logger(), "failed pos(x:%d, y:%d)", neighbor.x, neighbor.y);   ){ 
            //
               // RCLCPP_WARN(this->get_logger(), "8");
                continue;
                
                //obstacle check may not be working
            }
            //current_map_.data[(  neighbor.y   +adjustedHeight/2)*adjustedWidth +    (neighbor.x+adjustedWidth/2)] != 0 

            // neighbor.y = 
            //current_map_.info.resolution
           // RCLCPP_WARN(this->get_logger(), "9");
            
            double temp_g_score = (current.f_score-h_score(current.index,end)) +1;//current g score +1
            double temp_h_score = h_score(neighbor,end);

            
            //if new path to neighbor is shorter or neighbor is not in open
            if (open.find(neighbor) == open.end()){
                //RCLCPP_WARN(this->get_logger(), "adding to open list");
                AStarNode newNode(neighbor,temp_g_score+temp_h_score,current.index);//set f cost of neighbor and parent
                open.emplace(neighbor,newNode);
                //RCLCPP_WARN(this->get_logger(), "adding to open list");
                //RCLCPP_WARN(this->get_logger(), "10");
            }
            else if ((open.at(neighbor).f_score-temp_h_score) > temp_g_score){// new path is shorter
                //set f cost of neighbor, and add neighbor to open
                //open[neighbor] = AStarNode(neighbor,temp_g_score+h_score(neighbor,end),current.index);
                open.at(neighbor).f_score = temp_g_score+temp_h_score;
                open.at(neighbor).parentIndex = current.index;
                //RCLCPP_WARN(this->get_logger(), "11");
            }     
        }
        //for (const auto& [key,value] : open){//log open set
        //    RCLCPP_WARN(this->get_logger(), "open set pos(x:%d, y:%d), fscore:%lf", value.index.x, value.index.y,value.f_score);
        //}
        //for (const auto& [key,value] : close){//log closed set
        //    RCLCPP_WARN(this->get_logger(), "close set pos(x:%d, y:%d), fscore:%lf", value.index.x, value.index.y,value.f_score);
       // }
   }


       
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "sim_world";


            // Compute path using A* on current_map_
            // Fill path.poses with the resulting waypoints.
        RCLCPP_WARN(this->get_logger(), "publishing path");
        path_pub_->publish(path);
    }




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
