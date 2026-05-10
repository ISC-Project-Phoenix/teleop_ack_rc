cmake_minimum_required(VERSION 3.8)
project(teleop_ack_rc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 Dependencies 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) 
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find LibSerial 
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Include directories 
include_directories(include)

# --- 1. RC JOY NODE (Hardware Driver) --- 
add_library(rc_joy_node SHARED src/rc_joy_node.cpp)
ament_target_dependencies(rc_joy_node rclcpp sensor_msgs)
target_link_libraries(rc_joy_node ${LIBSERIAL_LIBRARIES})

add_executable(rc_joy src/rc_joy.cpp)
target_link_libraries(rc_joy rc_joy_node)

# --- 2. TELEOP ACK RC NODE (Logic) --- 
add_library(teleop_ack_rc_node SHARED src/teleop_ack_rc_node.cpp)
ament_target_dependencies(teleop_ack_rc_node rclcpp sensor_msgs ackermann_msgs)

add_executable(teleop_ack_rc src/teleop_ack_rc.cpp)
target_link_libraries(teleop_ack_rc teleop_ack_rc_node)

# --- Install Rules ---

# Install libraries to 'lib/' so the dynamic linker can find them
install(TARGETS
  rc_joy_node
  teleop_ack_rc_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables to 'lib/${PROJECT_NAME}' for ros2 run/launch
install(TARGETS
  rc_joy
  teleop_ack_rc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export the libraries so they are added to the library path
ament_export_libraries(rc_joy_node teleop_ack_rc_node)

ament_package()