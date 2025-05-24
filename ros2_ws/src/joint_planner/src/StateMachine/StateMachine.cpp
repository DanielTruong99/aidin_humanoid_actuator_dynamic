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
                /* initialize planner */
                _planner->reset();

                /* initialize actuator interface */
                // _actuator_interface->initialize();

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
            case Signal::EXIT_SIG:
            {
                /* initialize actuator interface */
                // _actuator_interface->initialize();

                /* transient to planning state */
                _state = (fsm::FSM::StateHandler)&PlannerStateMachine::planning_state;

                status = fsm::Status::TRAN_STATUS;
                break;
            }
    
            case Signal::TIMEOUT_1S_SIG:
            {
                /* transient to planning state */
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
            case Signal::TIMEOUT_10MS_SIG:
            {
                /* compute joint commands */
                static std::vector<float> joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0};
                // joint_positions = _planner->compute<std::vector<float>>(0.01);

                // /* send joint commands */
                // _actuator_interface->send_command(joint_positions);

                status = fsm::Status::HANDLED_STATUS;
                break;
            }
        }

        return status;
    }
}