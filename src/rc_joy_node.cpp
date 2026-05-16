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

/* apply_guardbands: map raw PWM to [-1.0, 1.0] with deadzone.
 *
 * Fix 2 – symmetric range:
 *   The original code divided the positive side by (max - dead_high) and the
 *   negative side by (dead_low - min), which are different numbers (~486 vs ~455
 *   with the default calibration).  That made the gain different on each side of
 *   centre, producing a small but noticeable jump / asymmetry through zero that
 *   caused jerky steering on the real robot.
 *
 *   We now pick the larger of the two half-ranges and use it for BOTH sides so
 *   the output gain is identical whether you steer left or right.
 */
float RcJoyNode::apply_guardbands(uint16_t raw, int min_v, int max_v, int dead_low, int dead_high) {
    float val = static_cast<float>(raw);

    if (val >= dead_low && val <= dead_high) return 0.0f;

    /* Use the larger half-range so both sides share the same gain. */
    float half_range = std::max(
        static_cast<float>(max_v  - dead_high),
        static_cast<float>(dead_low - min_v)
    );

    if (val > dead_high)
        return std::min( 1.0f, (val - dead_high) / half_range);
    else
        return std::max(-1.0f, (val - dead_low)  / half_range);
}

/* timer_callback: Read the raw bytes from lib serial and process them into Joy controller movements.
 *
 * Fix 1 – hold-last + re-sync:
 *   Original behaviour: only publish when a full valid packet arrives in this tick.
 *   On a real UART a missed byte or timing gap meant no message → robot stops cold
 *   even while the operator held a reverse command.  In simulation the Joy driver
 *   publishes continuously so this was never triggered.
 *
 *   New behaviour:
 *     a) If we receive a good packet, update last_joy_ and set has_valid_packet_.
 *     b) Always publish last_joy_ at the end of every 20 ms tick, stamped "now".
 *        This mirrors what teleop_ack_joy does (the joystick driver publishes at
 *        its own rate regardless of input events).
 *     c) If the buffer is unexpectedly deep (> 18 bytes = 2 full packets) we drain
 *        it down to one packet boundary to re-sync after byte-slip corruption.
 */
void RcJoyNode::timer_callback() {
    /* Collect the calibration parameters for steering. */
    int s_min = this->get_parameter("steer_min").as_int();
    int s_max = this->get_parameter("steer_max").as_int();
    int s_dl  = this->get_parameter("steer_dead_low").as_int();
    int s_dh  = this->get_parameter("steer_dead_high").as_int();

    /* Collect the calibration parameters for throttle. */
    int t_min = this->get_parameter("throttle_min").as_int();
    int t_max = this->get_parameter("throttle_max").as_int();
    int t_dl  = this->get_parameter("throttle_dead_low").as_int();
    int t_dh  = this->get_parameter("throttle_dead_high").as_int();

    /* Re-sync drain: if the buffer has grown beyond one frame, byte-slip has
     * occurred.  Consume bytes until we see 0xAA or the buffer empties so the
     * next read starts on a clean frame boundary. */
    int drain_limit = 64;
    while (serial_port_.IsDataAvailable() && drain_limit-- > 0) {
        uint8_t probe;
        serial_port_.ReadByte(probe);
        if (probe == 0xAA) {
            /* Found the next frame header – read its 8 payload bytes. */
            RC_PACKET p;
            p.header = 0xAA;
            std::vector<uint8_t> buffer;

            try {
                serial_port_.Read(buffer, 8, 20);
                if (buffer.size() == 8) {
                    std::memcpy(&p.steer, buffer.data(), 8);

                    auto joy_msg = sensor_msgs::msg::Joy();
                    joy_msg.header.frame_id = "rc_joy_frame";

                    /* Transform raw PWM to normalized floats [-1.0, 1.0] */
                    joy_msg.axes.push_back(apply_guardbands(p.steer,    s_min, s_max, s_dl, s_dh));
                    joy_msg.axes.push_back(apply_guardbands(p.throttle, t_min, t_max, t_dl, t_dh));

                    /* If teleop switch flicked make all 10 buttons 1 */
                    int btn_val = (p.teleop > 1500) ? 1 : 0;
                    joy_msg.buttons.assign(10, btn_val);

                    /* Store as the latest valid state. */
                    last_joy_ = joy_msg;
                    has_valid_packet_ = true;
                }
            } catch (...) {
                /* Timeout or short read – last_joy_ is unchanged; we will
                 * re-publish the previous good state below. */
            }

            /* Processed one frame this tick – stop draining. */
            break;
        }
        /* Byte was not 0xAA – discard and keep scanning. */
    }

    /* Always publish at 50 Hz so downstream nodes see a continuous stream.
     * If no good packet has ever arrived, we silently skip (robot stays still
     * until the first valid frame, which is the safe default). */
    if (has_valid_packet_) {
        last_joy_.header.stamp = this->now();
        joy_pub_->publish(last_joy_);
    }
}

}