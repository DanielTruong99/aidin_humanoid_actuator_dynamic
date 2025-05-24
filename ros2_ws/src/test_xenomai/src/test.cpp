#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <alchemy/task.h>

#define TASK_PRIO 99
#define TASK_STACK_SIZE 0
#define TASK_MODE 0

RT_TASK xeno_task;

void xeno_task_func(void *arg) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("xenomai_ros2_node");
    rclcpp::WallRate loop_rate(10); // 10 Hz

    while (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Xenomai task running with ROS2!");
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
    int ret = rt_task_create(&xeno_task, "XenoTask", TASK_STACK_SIZE, TASK_PRIO, TASK_MODE);
    if (ret) {
        std::cerr << "Failed to create Xenomai task: " << strerror(-ret) << std::endl;
        return 1;
    }

    ret = rt_task_start(&xeno_task, &xeno_task_func, nullptr);
    if (ret) {
        std::cerr << "Failed to start Xenomai task: " << strerror(-ret) << std::endl;
        return 1;
    }

    pause(); // Wait forever
    return 0;
}