#ifndef RC_JOY_NODE_HPP_
#define RC_JOY_NODE_HPP_

#include <libserial/SerialPort.h> /* SerialPort, Open(), ReadByte(), Read() */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp> /*  sensor_msgs::msg::Joy container for axes and button layouts */

namespace teleop_ack_rc {

/* 
* RC PACKET 
* 9-Byte binary frame from STM32 
* mapped via std::memcpy() in timer_callback
* deadman is for PWM-Serial converter internal use only.
*/
struct __attribute__((packed)) RC_PACKET {
    uint8_t header;    /* 0xAA */
    uint16_t steer;    /* Channel 1 */
    uint16_t throttle; /* Channel 4 */
    uint16_t teleop;   /* Channel 5 */
    uint16_t deadman;  /* Channel 6 */
};
/* RcJoyNode: responsible for converting PWM values to joy like messages */
class RcJoyNode : public rclcpp::Node {
public:
    explicit RcJoyNode(const rclcpp::NodeOptions& options);
    virtual ~RcJoyNode();

private:
    void timer_callback();
    float apply_guardbands(uint16_t raw, int min_v, int max_v, int dead_low, int dead_high);
    LibSerial::SerialPort serial_port_; /* The physical UART interface */
    std::string port_name_;             /* Device path (will always be /dev/rc_bridge)*/
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::TimerBase::SharedPtr timer_; /* Drives at 50 Hz polling loop. */
};

} /* namespace teleop_ack_rc*/

#endif