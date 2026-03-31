#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "teleop_ack_rc/teleop_ack_rc_node.hpp"

/* main: entry point for the teleop_ack_rc node */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<teleop_ack_rc::TeleopAckRcNode>(rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}