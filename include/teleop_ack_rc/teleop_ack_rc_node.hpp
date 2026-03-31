#ifndef TELEOP_ACK_RC_NODE_HPP_
#define TELEOP_ACK_RC_NODE_HPP_

#include <ackermann_msgs/msg/ackermann_drive.hpp> /* Provides our ackerman signalling Speed (m/s) and angle (rad)*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp> /* Normalize our stick values [-1.0, 1.0]*/

namespace teleop_ack_rc {

/* TeleopAckRcNode: Convert our RC joy messages into ackerman values for drive_mode_switch */
class TeleopAckRcNode : public rclcpp::Node {
public:
    explicit TeleopAckRcNode(const rclcpp::NodeOptions& options);

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_pub_;

    double max_speed_;          /* Maximum speed we will publish [-6.7056, 6.7056]*/
    double max_steering_angle_; /* Maximum angle we will publish [-0.2733, 0.2733]*/
};

}  // namespace teleop_ack_rc

#endif