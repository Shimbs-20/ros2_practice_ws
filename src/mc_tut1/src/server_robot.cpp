#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"


class Addtwoints :public  rclcpp::Node {
    public:
    Addtwoints(): Node("Server_robot"){
        server = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", 
        std::bind(&Addtwoints::call_back_addtwoints, this, std::placeholders::_1, std::placeholders::_2)); // use placeholders for callback function
        RCLCPP_INFO(this->get_logger(), "Add two init services");
    }
    

    private:
    void call_back_addtwoints(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                          const example_interfaces::srv::AddTwoInts::Response::SharedPtr response){

                            response->sum = request->a + request->b;
                            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", int(request->a), int(request->b), int(response->sum));
    }
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server;

};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node =  std::make_shared<Addtwoints>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}  