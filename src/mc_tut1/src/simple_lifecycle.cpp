#include <functional>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <thread>


class Simple_lifenode : public rclcpp_lifecycle::LifecycleNode{

    public:
    explicit Simple_lifenode(const std::string &name, bool interproccessor_id = false) :
     rclcpp_lifecycle::LifecycleNode(
        name,
         rclcpp::NodeOptions().use_intra_process_comms(interproccessor_id)){

         }

    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state){

        sub_ = create_subscription<std_msgs::msg::String>("Chatter", 10, std::bind(&Simple_lifenode::callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(),"Life cycle node is called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &previous_state){
        sub_.reset();
        RCLCPP_INFO(get_logger(),"Life cycle node on shutdown()is called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
     
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &previous_state){
        sub_.reset();
        RCLCPP_INFO(get_logger(),"Life cycle node on cleanup()is called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state){
        LifecycleNode::on_activate(previous_state);
        RCLCPP_INFO(get_logger(),"Life cycle node on activate()is called");
        std::this_thread::sleep_for(std::chrono::seconds(5)); // wait for 5 seconds before returning &rtime)
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state){
        LifecycleNode::on_deactivate(previous_state);
        RCLCPP_INFO(get_logger(),"Life cycle node on deactivate()is called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void callback(const std_msgs::msg::String::SharedPtr msg){
        auto state = get_current_state();
        if (state.label() == "inactive"){
            return;
        }
        RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
    }    
    

    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    
};





int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<Simple_lifenode>("simple_lifenode");

    executor.add_node(node ->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;    

}