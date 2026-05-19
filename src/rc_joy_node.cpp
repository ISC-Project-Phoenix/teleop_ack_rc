#include "teleop_ack_rc/rc_joy_node.hpp"
#include <vector>
#include <cstring>
#include <algorithm>

namespace teleop_ack_rc {

/* RcJoyNode: Constructor for our Joynode */
RcJoyNode::RcJoyNode(const rclcpp::NodeOptions & options)
    : Node("rc_joy_node", options) {
    
    this->declare_parameter("port", "/dev/rc_bridge");     /* make parameter to listen to port /dev/rc_bridge as STM will always connect there */
    port_name_ = this->get_parameter("port").as_string();  
    
    /* Gaurdbands for our steering input */
    this->declare_parameter("steer_min", 1038);
    this->declare_parameter("steer_max", 1990);
    this->declare_parameter("steer_dead_low", 1493);
    this->declare_parameter("steer_dead_high", 1504);
    
    /* Gaurdbands for our steering input */
    this->declare_parameter("throttle_min", 1013);
    this->declare_parameter("throttle_max", 1940);
    this->declare_parameter("throttle_dead_low", 1492);
    this->declare_parameter("throttle_dead_high", 1504);

    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/rc_joy", 10); 

    try {
        serial_port_.Open(port_name_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        RCLCPP_INFO(this->get_logger(), "Opened port: %s", port_name_.c_str());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Serial Error: %s", e.what());
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&RcJoyNode::timer_callback, this));
}

/* RcJoyNode: Destructor closes node on completion. */
RcJoyNode::~RcJoyNode() {
    if (serial_port_.IsOpen()) serial_port_.Close();
}

/* apply_gaurdbands: calculte the deadzones on the controller based on the configured parameters */
float RcJoyNode::apply_guardbands(uint16_t raw, int min_v, int max_v, int dead_low, int dead_high) {
    float val = static_cast<float>(raw);

    if (val >= dead_low && val <= dead_high) return 0.0f;

    if (val > dead_high) {
        float range = static_cast<float>(max_v) - dead_high;
        float out = (val - dead_high) / range;
        return std::min(1.0f, out);
    } 
    else {
        float range = static_cast<float>(dead_low) - min_v;
        float out = (val - dead_low) / range;
        return std::max(-1.0f, out);
    }
}

/* timer_callback: Read the raw bytes from lib serial and processess them into Joy controller movements. */
void RcJoyNode::timer_callback() {
    /* Collect the calibration parameters for steering. */
    int s_min = this->get_parameter("steer_min").as_int();
    int s_max = this->get_parameter("steer_max").as_int();
    int s_dl = this->get_parameter("steer_dead_low").as_int();
    int s_dh = this->get_parameter("steer_dead_high").as_int();

    /* Collect the calibration parameters for throttle. */
    int t_min = this->get_parameter("throttle_min").as_int();
    int t_max = this->get_parameter("throttle_max").as_int();
    int t_dl = this->get_parameter("throttle_dead_low").as_int();
    int t_dh = this->get_parameter("throttle_dead_high").as_int();

    while (serial_port_.IsDataAvailable()) {
        uint8_t header;
        serial_port_.ReadByte(header);

        /* Packet header 0xAA starts a new frame. */
        if (serial_port_.IsDataAvailable()) {
        uint8_t header = 0;
        serial_port_.ReadByte(header);

        if (header == 0xAA) {
            RC_PACKET p{}; // Zeros out memory instantly
            p.header = header;
            std::vector<uint8_t> buffer;

            try {
                // LOWER the timeout to 2-5ms max so it never delays your 20ms loop!
                serial_port_.Read(buffer, 8, 2); 
                
                if (buffer.size() == 8) {
                    std::memcpy(&p.steer, buffer.data(), 8);

                    auto joy_msg = sensor_msgs::msg::Joy();
                    joy_msg.header.stamp = this->now();
                    joy_msg.header.frame_id = "rc_joy_frame";

                    joy_msg.axes.push_back(apply_guardbands(p.steer, s_min, s_max, s_dl, s_dh));
                    joy_msg.axes.push_back(apply_guardbands(p.throttle, t_min, t_max, t_dl, t_dh));

                    int btn_val = (p.teleop > 1500) ? 1 : 0;
                    joy_msg.buttons.assign(10, btn_val);

                    joy_pub_->publish(joy_msg);
                }
            } catch (const std::exception & e) {
                // Avoid logging heavy ROS_ERROR messages inside 50Hz loops as it increases lag
            }
        }
    }
}

} 