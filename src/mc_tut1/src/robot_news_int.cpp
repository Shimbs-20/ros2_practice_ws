#include "example_interfaces/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotnewInt: public rclcpp::Node{
    public:
    RobotnewInt(): Node("Robot_News_Int"), count(0){
        publisher = this->create_publisher<example_interfaces::msg::Int32>("robot_news_int", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&RobotnewInt::publishnews, this));
        RCLCPP_INFO(this->get_logger(), "Robot news station started");
    }

    void publishnews(){
        auto msg = example_interfaces::msg::Int32();
        msg.data = count;
        publisher->publish(msg);
        count++;
    }

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr publisher;
    int count;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotnewInt>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}