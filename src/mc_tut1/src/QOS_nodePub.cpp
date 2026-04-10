#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <chrono>
#include <rmw/types.h>

class Qosnodepub : public rclcpp::Node {

public:
  Qosnodepub(const std::string &node_name = "SImple_qos_publisher")
      : Node(node_name), qos_profilepub(10), count(0) {
    declare_parameter<std::string>("reliability", "system_default");
    declare_parameter<std::string>("durability", "system_default");

    const auto reliability = this->get_parameter("reliability").as_string();
    const auto durability = this->get_parameter("durability").as_string();

    if (reliability == "best_effort") {
      qos_profilepub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      RCLCPP_INFO(this->get_logger(), "reliability is best effort");
    } else if (reliability == "RELIABLE") {
      qos_profilepub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      RCLCPP_INFO(this->get_logger(), "reliability is reliable");
    } else if (reliability == "system_default") {
      qos_profilepub.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
      RCLCPP_INFO(this->get_logger(), "reliability is system default");
    } else {
      RCLCPP_INFO(this->get_logger(), "reliability is invalid");
    }

    if (durability == "transient_local") {
      qos_profilepub.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      RCLCPP_INFO(this->get_logger(), "durability is transient local");
    } else if (durability == "volatile") {
      qos_profilepub.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      RCLCPP_INFO(this->get_logger(), "durability is volatile");
    } else if (durability == "system_default") {
      qos_profilepub.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
      RCLCPP_INFO(this->get_logger(), "durability is system default");
    } else {
      RCLCPP_INFO(this->get_logger(), "durability is invalid");
    }

    publisher = this->create_publisher<std_msgs::msg::String>("QOS_node_pub",
                                                              qos_profilepub);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&Qosnodepub::publishnews, this));
    RCLCPP_INFO(this->get_logger(), "Q0S node station started");
  }

  void publishnews() {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello " + std::to_string(count);
    publisher->publish(msg);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::QoS qos_profilepub;
  int count;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<Qosnodepub>("QOS_node_sub");
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}