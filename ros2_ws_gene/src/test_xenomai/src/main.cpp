#include <stdio.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>
#include <thread>
#include <atomic>
/* ---- Basic setting of Xenomai start ---- */
#include <errno.h>
#include <sys/mman.h>
#include <signal.h>

#include <alchemy/task.h>
#include <alchemy/sem.h>
#include <alchemy/mutex.h>
#include <alchemy/timer.h>
#include <trank/rtdk.h>
#include <rtdm/ipc.h>

// ROS2 headers
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"



void task_func(void *arg) 
{
    RTIME now, previous;
    rt_task_set_periodic(NULL, TM_NOW, 1e9);
    previous = rt_timer_read();
    
    while(true)
    {
        rt_task_wait_period(NULL);
        now = rt_timer_read();
        rt_printf("%s\n", "Hello from Xenomai task");

        previous = now;
    }
}

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char **argv)
{
    
    RT_TASK task;
    rt_task_create(&task, "SimpleTask", 0, 50, 0);
    rt_task_start(&task, &task_func, nullptr);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());

    rclcpp::shutdown();
    rt_task_delete(&task);

    return 0;
}