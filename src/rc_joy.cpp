#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "teleop_ack_rc/rc_joy_node.hpp"

/* main: main entry point for our joy node */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<teleop_ack_rc::RcJoyNode>(rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}