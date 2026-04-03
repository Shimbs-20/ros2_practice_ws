#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "my_robot_interfaces/msg/hardwarestatus.hpp"
using namespace std::chrono_literals;

class HardwarePublisher : public rclcpp::Node
{
public:
  HardwarePublisher(): Node("Hardware_Publisher")
  {
    Hardware_publisher_ =
      this->create_publisher<my_robot_interfaces::msg::Hardwarestatus>("Hardware_status", 10);
      RCLCPP_INFO(this->get_logger(), "Robot news station started");


    auto publish_msg = [this]() -> void {
        auto message = my_robot_interfaces::msg::Hardwarestatus();

        message.tempreture = 35.5;
        message.motors_ready = true;
        message.debug_message = "No error";
        RCLCPP_INFO(this->get_logger(), "Robot news station started %.2f", message.tempreture);


        this->Hardware_publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<my_robot_interfaces::msg::Hardwarestatus>::SharedPtr Hardware_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node =  std::make_shared<HardwarePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}