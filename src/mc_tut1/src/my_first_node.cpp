#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

class Mynode: public rclcpp::Node
{
    public:
    Mynode(): Node("my_first_node"), counter(0){
        RCLCPP_INFO(this->get_logger(), "Hello world ");
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Mynode::timer_callback, this));
    }
    void timer_callback(){
        RCLCPP_INFO(this->get_logger(), "Hello Shimbs %d", counter);
        counter++;
    }   
    private:
    rclcpp::TimerBase::SharedPtr timer_;
    int counter;

};


int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node =  std::make_shared<Mynode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}