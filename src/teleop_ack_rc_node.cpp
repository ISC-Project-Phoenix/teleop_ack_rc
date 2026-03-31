#include "teleop_ack_rc/teleop_ack_rc_node.hpp"

#include <algorithm> /* clamp, min, max */

namespace teleop_ack_rc {

/* TeleopAckRcNode: set the parameters for our ackerman and topics */
TeleopAckRcNode::TeleopAckRcNode(const rclcpp::NodeOptions& options) : Node("teleop_ack_rc_node", options) {
    this->declare_parameter("max_speed", 4.0);  // Max speed in m/s (adjust based on your vehicle's capabilities)
    this->declare_parameter("max_steering_angle", 0.2733);

    max_speed_ = this->get_parameter("max_speed").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

    ack_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/ack_vel", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/rc_joy", 10, std::bind(&TeleopAckRcNode::joy_callback, this, std::placeholders::_1));
}

/* joy_callback: convet our controller joy input into usefull ackerman values */
void TeleopAckRcNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.size() < 2) return;

    auto ack_msg = ackermann_msgs::msg::AckermannDrive();

    ack_msg.steering_angle = msg->axes[0] * -max_steering_angle_;
    ack_msg.speed = msg->axes[1] * max_speed_;

    ack_pub_->publish(ack_msg);
}

}  // namespace teleop_ack_rc