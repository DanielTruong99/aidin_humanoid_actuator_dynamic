cmake_minimum_required(VERSION 3.8)
project(joint_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rmw REQUIRED)

# build subdirectories
add_subdirectory(src/ActuatorInterface)
add_subdirectory(src/DataInterface)
add_subdirectory(src/Planner)
add_subdirectory(src/EventManager)
add_subdirectory(src/FSM)
add_subdirectory(src/StateMachine)

# build executables
add_executable(joint_planner src/joint_planner_node.cpp)
ament_target_dependencies(joint_planner
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(joint_planner
  actuator_interface
  data_interface
  fsm
  event_manager
  planner_sm
  planner
)

# install executables
install(TARGETS joint_planner
  DESTINATION 
)

install(DIRECTORY motion_files
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
