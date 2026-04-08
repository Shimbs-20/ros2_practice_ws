#include "/home/youssef-shemela/ros2_ws/src/ballbot_controller/include/ballbot_controller/simple_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/logging.hpp>
#include <eigen3/Eigen/Geometry>


Simplecontroller::Simplecontroller(const std::string &name): Node(name){

  wheel_radius_ = declare_parameter("wheel_radius", 0.033);
  wheel_base_ = declare_parameter("wheel_base", 0.17);

  RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
  RCLCPP_INFO_STREAM(get_logger(), "Using wheel base " << wheel_base_);

  sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/ballbot/cmd_vel", 10,
     std::bind(&Simplecontroller::callback, this, std::placeholders::_1));


  pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);

  pose_<< wheel_radius_/2, wheel_radius_/2,
          wheel_radius_/wheel_base_, -wheel_radius_/wheel_base_;

  RCLCPP_INFO_STREAM(get_logger(), "The convertion matrix " << pose_);

}


void Simplecontroller::callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
  std_msgs::msg::Float64MultiArray wheel_speed_msg;

  Eigen::Vector2d vel(msg->twist.linear.x, msg->twist.angular.z);
  Eigen::Vector2d wheel_vel = pose_.inverse() * vel;

  wheel_speed_msg.data.push_back(wheel_vel[0]);
  wheel_speed_msg.data.push_back(wheel_vel[1]);

  pub_->publish(wheel_speed_msg);
}



int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Simplecontroller>("simple_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}