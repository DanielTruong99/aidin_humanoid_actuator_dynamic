#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <dirent.h>
#include <chrono>
#include <array>

#include "../DataInterface/ROSDataInterface.h"

namespace planner 
{
    struct JointTrajectory
    {
        std::vector<std::vector<float>> joint_positions;
        std::vector<float> time;
    };

    using JointTrajectory = struct JointTrajectory;

    class Planner 
    {
        public:
            Planner(std::shared_ptr<ROSDataInterface> robot_interface) : 
                _robot_interface(robot_interface)
            {
                _initialize();
                _joint_positions_cmd.resize(5);
                _filtered_position_cmds.resize(5);
                constexpr float pi = 3.14159265358979323846f;
                _offset_positions = {0.0f, 0.0f, pi/4, -pi/2, pi/4}; // Joint position offset
            }

        private:
            /* Keep track time in the loaded motion file */
            float _current_time;
            std::vector<float> _offset_positions;   
        public:
            /* Compute the joint position cmds from the trajectory */
            std::vector<float> compute(float dt);

            /* Reset the planner */
            void reset();
            
            /* Check if the first joint position cmds is reached */
            bool is_reached() const;

            /* Check if the trajectory is completed */
            bool is_trajectory_completed() const;
            

            /* Change the index of trajectory */
            void change_trajectory();

            /* Get the current joint position cmds */
            std::vector<float> get_actions() const { return _joint_positions_cmd; }


        private:
            /* Variable related low pass filter */
            std::vector<float> _filtered_position_cmds;
            bool _is_first_time = true;
            const float _alpha = 0.8f;
        public:
            /* Apply low pass filter to the joint position commands */
            std::vector<float> apply_filter(std::vector<float> &joint_positions_cmds);

            /* Reset the filter */
            void reset_filter();


        private:
            /* Robot interface */
            std::shared_ptr<ROSDataInterface> _robot_interface;

            
        private:
            /* Joint trajectory data */
            std::vector<float> _joint_positions_cmd;
            std::vector<JointTrajectory> _joint_trajectories;
            std::array<bool, 100> _joint_trajectory_keys;
            std::vector<std::string> _joint_trajectory_files;
            uint8_t _current_trajectory_index = 0;

            /* Enumerate motion files in specified directory*/
            void _initialize();

            /* Load trajectory from file */
            void _load_trajectory(const std::string &file_path);
    };

} // namespace Planner

#endif // PLANNER_H