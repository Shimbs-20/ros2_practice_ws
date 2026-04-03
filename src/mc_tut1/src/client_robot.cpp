#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <rclcpp/client.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/logging.hpp>


class Addtwointsclient :public  rclcpp::Node {
    public:
    Addtwointsclient(): Node("client_robot"){
        client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    }

    void call_back_addtwoints(int a, int b){
        while(!client->wait_for_service()){
            RCLCPP_INFO(this->get_logger(), "waiting for server");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        auto result = client->async_send_request(request, std::bind(&Addtwointsclient::callback, this, std::placeholders::_1));
        if(result.get()->sum == 0){
            RCLCPP_INFO(this->get_logger(), "Error");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Sum , %d",(int)result.get()->sum);
        }
    }

    private:
    void callback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future){
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(),"Sum , %d",(int)response->sum);
    }
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node =  std::make_shared<Addtwointsclient>();
    node->call_back_addtwoints(10, 15);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}  