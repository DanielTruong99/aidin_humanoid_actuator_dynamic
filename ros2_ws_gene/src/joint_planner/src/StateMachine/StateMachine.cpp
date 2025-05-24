#include "StateMachine.h"

namespace planner_sm
{
    Status PlannerStateMachine::initial_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Initial state");
                _start_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_3S_SIG:
            {
                /* transient to configuration state */
                _state = (fsm::FSM::StateHandler)&PlannerStateMachine::configuration_state;
                status = fsm::Status::TRAN_STATUS;
                break;
            }
            
        }

        return status;
    }

    Status PlannerStateMachine::configuration_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Configuration state");
                _start_timer();
                _planner->compute(_sampling_time); // call the compute to get the first position in the trajectory, saved internally
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_5MS_SIG:
            {
                // if(!_planner->is_reached())
                // {
                //     static bool is_first_time = true;
                //     if(is_first_time)
                //     {
                //         _planner->reset_filter();
                //         is_first_time = false;
                //     }
                //     std::vector<float> joint_positions_cmds = _planner->get_actions();
                //     std::vector<float> filtered_positions_cmds = _planner->apply_filter(joint_positions_cmds);

                //     /* Check valid joint cmds */
                //     bool is_nan = std::any_of(filtered_positions_cmds.begin(), filtered_positions_cmds.end(), [](float val) { return std::isnan(val); });
                //     bool is_inf = std::any_of(filtered_positions_cmds.begin(), filtered_positions_cmds.end(), [](float val) { return std::isinf(val); });
                //     if(is_nan || is_inf)
                //     {
                //         RCLCPP_ERROR(_node->get_logger(), "PlannerStateMachine: Joint position commands contain NaN or Inf values");
                //         status = fsm::Status::IGNORED_STATUS;
                //         break;
                //     }

                //     /* Send the filtered command to the actuator interface */
                //     _actuator_interface->send_command(filtered_positions_cmds);
                //     _actuator_interface->send_command_raw(joint_positions_cmds);

                //     status = fsm::Status::HANDLED_STATUS;
                //     break;
                // }

                /* First configuration of the Planner plan is reached, transient to planning state*/
               _state = (fsm::FSM::StateHandler)&PlannerStateMachine::planning_state;
                status = fsm::Status::TRAN_STATUS;
                break;
            }

        }

        return status;
    }

    Status PlannerStateMachine::planning_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Planning state");
                _start_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_5MS_SIG:
            {
                if(_planner->is_trajectory_completed())
                {
                    /* Planner plan for 1 trajectory is reached, transient to finished state */
                    _state = (fsm::FSM::StateHandler)&PlannerStateMachine::finished_state;
                    status = fsm::Status::TRAN_STATUS;
                    break;
                }

                /* Planning
                    Compute joint commands and send joint commands
                */
                static bool is_first_time = true;
                if(is_first_time)
                {
                    _planner->reset_filter();
                    is_first_time = false;
                }
                std::vector<float> joint_position_cmds = _planner->compute(_sampling_time);
                std::vector<float> filtered_positions_cmds = _planner->apply_filter(joint_position_cmds);

                /* Check valid joint cmds */
                bool is_nan = std::any_of(filtered_positions_cmds.begin(), filtered_positions_cmds.end(), [](float val) { return std::isnan(val); });
                bool is_inf = std::any_of(filtered_positions_cmds.begin(), filtered_positions_cmds.end(), [](float val) { return std::isinf(val); });
                if(is_nan || is_inf)
                {
                    RCLCPP_ERROR(_node->get_logger(), "PlannerStateMachine: Joint position commands contain NaN or Inf values");
                    status = fsm::Status::IGNORED_STATUS;
                    break;
                }

                /* Send the filtered command to the actuator interface */
                _actuator_interface->send_command(filtered_positions_cmds);
                _actuator_interface->send_command_raw(joint_position_cmds);

                status = fsm::Status::HANDLED_STATUS;
                break;
            }
        }

        return status;
    }

    Status PlannerStateMachine::finished_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
        case Signal::ENTRY_SIG:
        {
            RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Finished state");
            _start_timer();
            _planner->change_trajectory();
            status = fsm::Status::HANDLED_STATUS;
            break;
        }

        case Signal::EXIT_SIG:
        {
            _stop_timer();
            status = fsm::Status::HANDLED_STATUS;
            break;
        }

        case Signal::TIMEOUT_1S_SIG:
        {
            /* Transient to configuration state */
            _state = (fsm::FSM::StateHandler)&PlannerStateMachine::configuration_state;
            status = fsm::Status::TRAN_STATUS;
            break;
        }
        }

        return status;
    }
}