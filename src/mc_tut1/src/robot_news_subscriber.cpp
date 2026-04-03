#include "example_interfaces/msg/string.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <rclcpp/subscription.hpp>

class Smartphonenode :public  rclcpp::Node {
    public:
    Smartphonenode(): Node("Smart_subscriber"){
        subscriber = this->create_subscription<example_interfaces::msg::String>("robot_news", 10, std::bind(&Smartphonenode::callback, this, std::placeholders::_1));
        // add place holder for callback function qos is quality of service so 10 represents 10 messages at a time to be processed
        RCLCPP_INFO(this->get_logger(), "Smartphone has been started");
        //RCLCPP_INFO(this->get_logger(), "Smartphone has been started"); is for logging at first initialization
    }
    
    void callback(const example_interfaces::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str()); //use .c_str() to convert string to char to work with %s printf
    }
    private:
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node =  std::make_shared<Smartphonenode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}  