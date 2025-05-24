#include "ROSActuatorInterface.h"

ROSActuatorInterface::ROSActuatorInterface(rclcpp::Node::SharedPtr node)
: _node(node)
{
    /* Initialize the publisher and message */
    _joint_state_publisher = _node->create_publisher<sensor_msgs::msg::JointState>("joint_cmd", 10);
    _joint_state_msg.name = {"q1", "q2", "q3", "q4", "q5"}; 
    _joint_state_msg.position.resize(_joint_state_msg.name.size());
}

template <typename T>
void ROSActuatorInterface::send_command(T &joint_positions)
{
    _joint_state_msg.header.stamp = _node->now();
    _joint_state_msg.position = joint_positions.get_joint_positions();
    _joint_state_publisher->publish(_joint_state_msg);
}