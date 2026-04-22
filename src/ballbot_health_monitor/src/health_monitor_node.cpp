#include <map>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace {
constexpr double WARN_TIMEOUT_S = 2.0;
constexpr double ERROR_TIMEOUT_S = 5.0;

std::pair<unsigned char, std::string>
classify_timeout(const std::optional<double> &last_seen, double now,
                 double warn_threshold = WARN_TIMEOUT_S,
                 double error_threshold = ERROR_TIMEOUT_S) {
  if (!last_seen.has_value()) {
    return {DiagnosticStatus::ERROR, "No message received"};
  }
  const double age = now - last_seen.value();
  char buf[64];
  if (age > error_threshold) {
    std::snprintf(buf, sizeof(buf), "No message for %.2f s", age);
    return {DiagnosticStatus::ERROR, buf};
  }
  if (age > warn_threshold) {
    std::snprintf(buf, sizeof(buf), "No message for %.2f s", age);
    return {DiagnosticStatus::WARN, buf};
  }
  std::snprintf(buf, sizeof(buf), "Last message %.2f s ago", age);
  return {DiagnosticStatus::OK, buf};
}
}  // namespace

class HealthMonitorNode : public rclcpp::Node {
public:
  HealthMonitorNode() : Node("health_monitor_node") {
    add_topic<nav_msgs::msg::Odometry>("/odom");
    add_topic<sensor_msgs::msg::LaserScan>("/scan");
    add_topic<sensor_msgs::msg::Imu>("/imu");
    add_topic<sensor_msgs::msg::JointState>("/joint_states");
    add_topic<std_msgs::msg::Float64MultiArray>(
        "/simple_velocity_controller/commands");

    diag_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);

    timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&HealthMonitorNode::publish_diagnostics, this));

    RCLCPP_INFO(get_logger(), "Monitoring %zu topics", last_seen_.size());
  }

private:
  template <typename MsgT>
  void add_topic(const std::string &topic) {
    last_seen_[topic] = std::nullopt;
    auto cb = [this, topic](const typename MsgT::SharedPtr) {
      last_seen_[topic] = now_seconds();
    };
    subs_.push_back(create_subscription<MsgT>(topic, 10, cb));
  }

  double now_seconds() const {
    return get_clock()->now().seconds();
  }

  void publish_diagnostics() {
    const double now = now_seconds();
    DiagnosticArray arr;
    arr.header.stamp = get_clock()->now();

    for (const auto &[topic, last_seen] : last_seen_) {
      auto [level, message] = classify_timeout(last_seen, now);
      DiagnosticStatus status;
      status.level = level;
      status.name = std::string(get_name()) + ": " + topic;
      status.message = message;
      status.hardware_id = topic;
      KeyValue kv;
      kv.key = "age_s";
      kv.value = last_seen ? std::to_string(now - *last_seen) : "never";
      status.values.push_back(kv);
      arr.status.push_back(status);
    }

    diag_pub_->publish(arr);
  }

  std::map<std::string, std::optional<double>> last_seen_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HealthMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
