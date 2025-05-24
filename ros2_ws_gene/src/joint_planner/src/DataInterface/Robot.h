#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H
#pragma once

class Robot 
{
    public:
        Robot()
        {
            _joint_positions = std::vector<float>(5, 0.0f);
            _joint_velocities = std::vector<float>(5, 0.0f);
            _joint_torques = std::vector<float>(5, 0.0f);
            _vb = std::vector<float>(3, 0.0f);
            _wb = std::vector<float>(3, 0.0f);
            _rw = std::vector<float>(3, 0.0f);
            _qw = std::vector<float>(4, 0.0f);
        };

    public:
        /* Getters */
        std::vector<float> get_joint_positions() const { return _joint_positions; }
        std::vector<float> get_joint_velocities() const { return _joint_velocities; }
        std::vector<float> get_joint_torques() const { return _joint_torques; }
        std::vector<float> get_base_velocity() const { return _vb; }
        std::vector<float> get_base_angular_velocity() const { return _wb; }
        std::vector<float> get_base_position() const { return _rw; }
        std::vector<float> get_base_quaternion() const { return _qw; }

    protected:
        /* Low level states */
        std::vector<float> _joint_positions;
        std::vector<float> _joint_velocities;
        std::vector<float> _joint_torques;

    protected:
        /* High level states */
        std::vector<float> _vb; // Base velocity in base frame
        std::vector<float> _wb; // Base angular velocity in base frame
        std::vector<float> _rw; // Base position in world frame
        std::vector<float> _qw; // Base quaternion in world frame
};

#endif // ROBOT_INTERFACE_H

