cmake_minimum_required(VERSION 3.5)
project(non_rt_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

#



add_executable(motor_state_publisher src/motor_state_publisher.cpp)
ament_target_dependencies(motor_state_publisher
  rclcpp
  sensor_msgs
)
target_link_libraries(motor_state_publisher rt)

add_executable(motor_command_subriber src/motor_command_subriber.cpp)
ament_target_dependencies(motor_command_subriber
  rclcpp
  sensor_msgs
)
target_link_libraries(motor_command_subriber rt)

# install
install(TARGETS motor_command_subriber motor_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
