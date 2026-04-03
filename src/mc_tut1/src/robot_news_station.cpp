#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <example_interfaces/msg/string.hpp>
#include <array>

class Robotnewsstation : public rclcpp::Node
{
public:
    public:
    Robotnewsstation(): Node("Robot_News_Station"), counter(0){
        this->declare_parameter("number", 20);
        this->declare_parameter("timer_period", 0.5);
        number = this->get_parameter("number").as_int();
        timer1 = this->get_parameter("timer_period").as_double();

        publisher = this->create_publisher<example_interfaces::msg::String>("robot_news",10);
        timer = this->create_wall_timer(std::chrono::duration<double>(timer1), std::bind(&Robotnewsstation::publishnews, this));
        RCLCPP_INFO(this->get_logger(), "Robot news station started");
    }

    void publishnews(){
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hello world! ") + std::to_string(counter) + " from "+ std::to_string(number);
        publisher->publish(msg);
        counter++;
    }    
    private:
    rclcpp::Publisher<example_interfaces::msg::String>:: SharedPtr publisher;
    std::array<example_interfaces::msg::String, 10> robot_name;
    int counter;
    int number;
    double timer1;
    rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node =  std::make_shared<Robotnewsstation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;}