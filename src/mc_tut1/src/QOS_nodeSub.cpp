#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <chrono>
#include <rclcpp/subscription.hpp>
#include <rmw/types.h>

class Qosnodesub : public rclcpp::Node {

public:
  Qosnodesub(const std::string &node_name = "Simple_qos_sub")
      : Node(node_name), qos_profilesub(10), count(0) {
    declare_parameter<std::string>("reliability", "system_default");
    declare_parameter<std::string>("durability", "system_default");

    const auto reliability = this->get_parameter("reliability").as_string();
    const auto durability = this->get_parameter("durability").as_string();

    if (reliability == "best_effort") {
      qos_profilesub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      RCLCPP_INFO(this->get_logger(), "reliability is best effort");
    } else if (reliability == "RELIABLE") {
      qos_profilesub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      RCLCPP_INFO(this->get_logger(), "reliability is reliable");
    } else if (reliability == "system_default") {
      qos_profilesub.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
      RCLCPP_INFO(this->get_logger(), "reliability is system default");
    } else {
      RCLCPP_INFO(this->get_logger(), "reliability is invalid");
    }

    if (durability == "transient_local") {
      qos_profilesub.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      RCLCPP_INFO(this->get_logger(), "durability is transient local");
    } else if (durability == "volatile") {
      qos_profilesub.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      RCLCPP_INFO(this->get_logger(), "durability is volatile");
    } else if (durability == "system_default") {
      qos_profilesub.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
      RCLCPP_INFO(this->get_logger(), "durability is system default");
    } else {
      RCLCPP_INFO(this->get_logger(), "durability is invalid");
    }

    subscriber = this->create_subscription<std_msgs::msg::String>(
        "QOS_node_pub", qos_profilesub,
        std::bind(&Qosnodesub::callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Q0S node station started");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
  rclcpp::QoS qos_profilesub;
  size_t count = 0;
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    count++;
    RCLCPP_INFO(this->get_logger(), "I heard: '%s' and counter is %ld", msg->data.c_str(), count);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<Qosnodesub>("QOS_node_pub");
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}