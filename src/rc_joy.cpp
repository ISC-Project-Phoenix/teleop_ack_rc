#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "teleop_ack_rc/rc_joy_node.hpp"

/* main: main entry point for our joy node */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    
    auto node = std::make_shared<teleop_ack_rc::RcJoyNode>(options);
    exec.add_node(node);
    
    exec.spin();
    rclcpp::shutdown();
    return 0;
}