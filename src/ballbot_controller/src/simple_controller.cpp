
#include "/home/youssef-shemela/ros2_ws/src/ballbot_controller/include/ballbot_controller/simple_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/logging.hpp>
#include <eigen3/Eigen/Geometry>


Simplecontroller::Simplecontroller(const std::string &name): Node(name){

  wheel_radius_ = declare_parameter("wheel_radius", 0.033);
  wheel_base_ = declare_parameter("wheel_base", 0.17);

  RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
  RCLCPP_INFO_STREAM(get_logger(), "Using wheel base " << wheel_base_);

  sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
     std::bind(&Simplecontroller::callback, this, std::placeholders::_1));


  pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_vel", 10);

  pose_<< wheel_radius_/2, wheel_radius_/2,
          wheel_radius_/wheel_base_, -wheel_radius_/wheel_base_;

  RCLCPP_INFO_STREAM(get_logger(), "The convertion matrix " << pose_);

}


void Simplecontroller::callback(const geometry_msgs::msg::Twist::SharedPtr msg){
  auto msg_vel = std_msgs::msg::Float64MultiArray();

  Eigen::Vector2d vel(msg->linear.x, msg->linear.y);
  Eigen::Vector2d wheel_vel = pose_.inverse() * vel;

  msg_vel.data.push_back(wheel_vel[0]);
  msg_vel.data.push_back(wheel_vel[1]);

  pub_->publish(msg_vel);
}



int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Simplecontroller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}