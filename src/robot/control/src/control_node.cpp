#include <cmath>
#include <optional>

#include  <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Initialize parameters
  lookahead_distance_ = 1;
  //goal_tolerance_ = 2;   
  linear_speed_ = 0.5;       

  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });

  // Data
 // nav_msgs::msg::Path::SharedPtr current_path_;
 // nav_msgs::msg::Odometry::SharedPtr robot_odom_;

  // Parameters

}

void ControlNode::controlLoop(){
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_) {
      RCLCPP_WARN(this->get_logger(), "0");
      stopMoving();
      return;
  }

  // Find the lookahead point
  //RCLCPP_WARN(this->get_logger(), "1");
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
   // RCLCPP_WARN(this->get_logger(), "4");
    stopMoving();
    return;  // No valid lookahead point found
  }
//RCLCPP_WARN(this->get_logger(), "5");
  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);
  RCLCPP_WARN(this->get_logger(), "velocities computed");
  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
  RCLCPP_WARN(this->get_logger(), "velocities published");
}

void ControlNode::stopMoving(){
  geometry_msgs::msg::Twist zero_vel;
  zero_vel.linear.x = 0;
  zero_vel.linear.y=0;
  zero_vel.linear.z=0;
  zero_vel.angular.x=0;
  zero_vel.angular.y=0;
  zero_vel.angular.z=0;


  cmd_vel_pub_->publish(zero_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  // TODO: Implement logic to find the lookahead point on the path
  
  /*
    nav_msgs::msg::Path is an array of geometry_msgs::msg::PoseStamped. Loop through each point and
    computeDistance() between point and robot location (from odometry) until we ger distance closest to lookahead_distance
  */
    RCLCPP_WARN(this->get_logger(), "2");
    RCLCPP_WARN(this->get_logger(), "path array size:%d ",current_path_->poses.size());
    for (int i = current_path_->poses.size() - 1; i >= 0; i--) { 
      RCLCPP_WARN(this->get_logger(), "distance: %lf",computeDistance(robot_odom_->pose.pose.position, current_path_->poses[i].pose.position));
      RCLCPP_WARN(this->get_logger(), "3");
      if (computeDistance(robot_odom_->pose.pose.position, current_path_->poses[i].pose.position)  < lookahead_distance_){// goal_tolerance_) {//within on then select that node
          RCLCPP_WARN(this->get_logger(), "3.5");
          return current_path_->poses[i];
      }
    }
  
  
  return std::nullopt;  // Replace with a valid point when implemented // -- or keep as nullopt if cant find a path
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  // TODO: Implement logic to compute velocity commands
  geometry_msgs::msg::Twist cmd_vel;
  
  //calculate linear velocity(it only moves forward in this case)
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;

  //calculate angular velocity
  float currentYaw = extractYaw(robot_odom_->pose.pose.orientation);
  float desiredYaw = atan2(target.pose.position.y-robot_odom_->pose.pose.position.y,
  target.pose.position.x-robot_odom_->pose.pose.position.x);
  //float YawError = desiredYaw-currentYaw;
  float YawError = fmod(desiredYaw - currentYaw + M_PI, 2 * M_PI) - M_PI;
  float kp =0.5;//tune the proportional gain
  float angularVelocity = YawError*kp;
  //float currentPos = odom_sub_.pose.pose.position.x;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0; 
  cmd_vel.angular.z = angularVelocity; 
  /*
    Using current orientation (from odom) and the position of the target, somehow find the angular velocity
      (linear velocity is fixed)

    Convert quarterion from odom to yaw to help

     1. Get robot's yaw
     2. Get position's
  */

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &msg_quat) {
  tf2::Quaternion quat(msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
