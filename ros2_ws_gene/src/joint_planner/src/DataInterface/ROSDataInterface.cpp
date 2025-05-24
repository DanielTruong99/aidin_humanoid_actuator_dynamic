#include "ROSDataInterface.h"

void ROSDataInterface::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    /* Check nan */
    bool is_pos_nan = std::any_of(msg->position.begin(), msg->position.end(), [](float val) { return std::isnan(val); });
    bool is_vel_nan = std::any_of(msg->velocity.begin(), msg->velocity.end(), [](float val) { return std::isnan(val); });
    if(is_pos_nan || is_vel_nan) return;

    /* Data are safe, cache the joint data */
    std::copy(msg->position.begin(), msg->position.end(), _joint_positions.begin());
    std::copy(msg->velocity.begin(), msg->velocity.end(), _joint_velocities.begin());
    std::copy(msg->effort.begin(), msg->effort.end() - 1, _joint_torques.begin());
}
