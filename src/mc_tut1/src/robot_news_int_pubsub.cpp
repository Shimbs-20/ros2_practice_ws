#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"
//how to include and use algorithm library
#include <algorithm>


class robot_news_station: public rclcpp::Node{
    public:
    robot_news_station(): Node("Robot_News_Station"){
        subscription = this->create_subscription<example_interfaces::msg::Int32>("robot_news_int", 10,
             std::bind(&robot_news_station::topic_callback, this, std::placeholders::_1)); // bind here makes the callback function run in parallel with the publisher
        publisher = this->create_publisher<example_interfaces::msg::Int32>("robot_news_int_number_added", 10);
        RCLCPP_INFO(this->get_logger(), "Robot news station started");
    }

    void topic_callback(const example_interfaces::msg::Int32::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
        add += msg->data ;
        auto message = example_interfaces::msg::Int32();
        message.data = add;
        publisher->publish(message);
    }

    private:
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr subscription;
    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr publisher;
    int add;    
};



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_news_station>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}