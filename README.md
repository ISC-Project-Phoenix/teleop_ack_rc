# Teleoperations Ackermann RC
**Project Phoenix | ROS 2 Control Interface**

This package provides a high-performance bridge between a hardware **PWM-to-Serial converter** (STM32-based) and the ROS 2 ecosystem. It serves as the primary hardware abstraction layer for RC-based teleoperation of an Ackermann-steered vehicle.

---

## 1. System Overview
The system is divided into two distinct functional layers to separate hardware communication from vehicle logic:

* **`rc_joy_node` (Hardware Driver):** Reads a raw 9-byte binary packet from the Serial/UART interface. It performs synchronization, maps raw PWM values to normalized floats, and publishes `sensor_msgs/msg/Joy`.
* **`teleop_ack_rc_node` (Logic Bridge):** Subscribes to the Joy topic and translates normalized axes into physical vehicle units (Meters per Second and Radians) for the `ackermann_msgs/msg/AckermannDrive` format.

## 2. File Structure
```text
teleop_ack_rc/
├── config/
│   └── params.yaml          # Calibration: PWM limits, deadzones, and max speed
├── include/
│   └── teleop_ack_rc/       # K&R Style documented C++ Headers
├── src/
│   ├── rc_joy.cpp           # Main wrapper for the Hardware Driver
│   ├── rc_joy_node.cpp      # Implementation of Serial/PWM logic
│   ├── teleop_ack_rc.cpp    # Main wrapper for the Logic Bridge
│   └── teleop_ack_rc_node.cpp # Implementation of Ackermann scaling
├── CMakeLists.txt           # Build instructions (LibSerial & ROS 2)
└── package.xml              # Dependency manifest
```

## 3. Dependencies
The following libraries must be installed on the host system:
* **`libserial-dev`**: C++ library for serial port communication.
  ```bash
  sudo apt-get install libserial-dev
