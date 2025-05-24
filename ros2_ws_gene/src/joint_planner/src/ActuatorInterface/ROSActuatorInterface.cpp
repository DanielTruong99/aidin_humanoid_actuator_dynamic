#include "ROSActuatorInterface.h"

ROSActuatorInterface::ROSActuatorInterface(rclcpp::Node::SharedPtr node)
: _node(node)
{
    /* Initialize the publisher and message */
    _filtered_action_publisher = _node->create_publisher<sensor_msgs::msg::JointState>("filtered_joint_cmd", 10);
    _filtered_action_msg.name = {"q1", "q2", "q3", "q4", "q5"}; 
    _filtered_action_msg.position.resize(_filtered_action_msg.name.size());

    /* Initialize the publisher and message */
    _action_publisher = _node->create_publisher<sensor_msgs::msg::JointState>("joint_cmd", 10);
    _action_msg.name = {"q1", "q2", "q3", "q4", "q5"}; 
    _action_msg.position.resize(_action_msg.name.size());
}

void ROSActuatorInterface::send_command(std::vector<float> &joint_positions)
{
    _filtered_action_msg.header.stamp = _node->now();
    std::copy(joint_positions.begin(), joint_positions.end(), _filtered_action_msg.position.begin());
    _filtered_action_publisher->publish(_filtered_action_msg);
}

/* For debugging only */
void ROSActuatorInterface::send_command_raw(std::vector<float> &joint_positions)
{
    _action_msg.header.stamp = _node->now();
    std::copy(joint_positions.begin(), joint_positions.end(), _action_msg.position.begin());
    _action_publisher->publish(_action_msg);
}