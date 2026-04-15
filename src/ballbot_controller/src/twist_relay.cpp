#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

class twsit_relay : public rclcpp::Node {
public:
  twsit_relay() : Node("twist_relay") {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/ballbot_controller/cmd_vel_unstamped", 10,
        std::bind(&twsit_relay::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "ballbot_controller/cmd_vel", 10);
    joy_stick_sub_ =
        this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/input_joy/cmd_vel_stamped", 10,
            std::bind(&twsit_relay::joy_twist_callback, this,
                      std::placeholders::_1)); // joy
    joy_stick_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/input_joy/cmd_vel", 10);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp ::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      joy_stick_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_stick_pub_;

  void callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = get_clock()->now();
    twist_stamped.twist = *msg;
    pub_->publish(twist_stamped);
  }

  void
  joy_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    geometry_msgs::msg::Twist joy_twist;
    joy_twist = msg->twist;
    joy_stick_pub_->publish(joy_twist);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<twsit_relay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}