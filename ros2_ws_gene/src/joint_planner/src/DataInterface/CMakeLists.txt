
add_library(data_interface STATIC ROSDataInterface.h Robot.h ROSDataInterface.cpp)

target_include_directories(data_interface PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(data_interface rclcpp std_msgs sensor_msgs)



