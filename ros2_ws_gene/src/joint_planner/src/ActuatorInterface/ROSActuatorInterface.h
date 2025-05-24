#ifndef ROS_ACTUATOR_INTERFACE_H
#define ROS_ACTUATOR_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"



class ROSActuatorInterface
{
    public:
        ROSActuatorInterface(rclcpp::Node::SharedPtr node);

        void send_command(std::vector<float> &joint_positions);
        void send_command_raw(std::vector<float> &joint_positions);

    private:
        rclcpp::Node::SharedPtr _node;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _filtered_action_publisher;
        sensor_msgs::msg::JointState _filtered_action_msg;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _action_publisher;
        sensor_msgs::msg::JointState _action_msg;
};

#endif // ROS_ACTUATOR_INTERFACE_H