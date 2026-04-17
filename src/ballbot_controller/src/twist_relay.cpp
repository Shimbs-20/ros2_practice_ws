#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
 
// BUG FIX 1: Class name typo "twsit_relay" → "twist_relay"
class twist_relay : public rclcpp::Node {
public:
  twist_relay() : Node("twist_relay") {
 
    // -----------------------------------------------------------------------
    // Path A: twist_mux → /ballbot_controller/cmd_vel_unstamped (Twist)
    //         stamp it → ballbot_controller/cmd_vel (TwistStamped)
    //         This is what diff_drive_controller reads when use_stamped_vel=true
    // -----------------------------------------------------------------------
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/ballbot_controller/cmd_vel_unstamped", 10,
        std::bind(&twist_relay::callback, this, std::placeholders::_1));
 
    // BUG FIX 2: Missing leading slash made this a relative topic
    // "ballbot_controller/cmd_vel" → "/ballbot_controller/cmd_vel"
    // diff_drive_controller subscribes on the absolute topic.
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ballbot_controller/cmd_vel", 10);
 
    // -----------------------------------------------------------------------
    // Path B (joystick strip): joy_teleop now publishes Twist directly on
    // /input_joy/cmd_vel so this subscriber/publisher pair is no longer
    // needed. Kept here commented out in case you revert joy_teleop to
    // TwistStamped mode.
    //
    // joy_stick_sub_ =
    //     this->create_subscription<geometry_msgs::msg::TwistStamped>(
    //         "/input_joy/cmd_vel_stamped", 10,
    //         std::bind(&twist_relay::joy_twist_callback, this,
    //                   std::placeholders::_1));
    // joy_stick_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    //     "/input_joy/cmd_vel", 10);
    // -----------------------------------------------------------------------
  }
 
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
 
  // Stamp an incoming Twist and forward as TwistStamped
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = get_clock()->now();
    twist_stamped.twist = *msg;
    pub_->publish(twist_stamped);
  }
};
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // BUG FIX 3: was instantiating old misspelled class name
  auto node = std::make_shared<twist_relay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}