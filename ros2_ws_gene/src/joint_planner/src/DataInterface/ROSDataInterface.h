#ifndef ROS_DATA_INTERFACE_H
#define ROS_DATA_INTERFACE_H

#include <vector>
#include "Robot.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ROSDataInterface : public Robot
{
    public:
        ROSDataInterface(rclcpp::Node::SharedPtr node) : Robot(), _node(node)
        {
            _joint_state_sub = _node->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&ROSDataInterface::joint_state_callback, this, std::placeholders::_1)
            );
        }
        
    private:
        rclcpp::Node::SharedPtr _node;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
    
    private: 
        /* Callback function for joint state subscriber */
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};



#endif // ROS_DATA_INTERFACE_H