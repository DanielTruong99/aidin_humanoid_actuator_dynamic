#ifndef ROS_ACTUATOR_INTERFACE_H
#define ROS_ACTUATOR_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"



class ROSActuatorInterface
{
    public:
        ROSActuatorInterface(rclcpp::Node::SharedPtr node);

        template <typename T>
        void send_command(T &joint_positions);

    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_state_publisher;
        rclcpp::Node::SharedPtr _node;
        sensor_msgs::msg::JointState _joint_state_msg;
};

#endif // ROS_ACTUATOR_INTERFACE_H