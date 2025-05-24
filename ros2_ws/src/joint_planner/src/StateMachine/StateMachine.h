
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <cstdint>
#include "rclcpp/rclcpp.hpp"

#include "ROSActuatorInterface.h"
#include "Planner.h"
#include "EventManager.h"
#include "FSM.h"


namespace planner_sm
{

    using namespace fsm;
    using namespace event_manager;
    class PlannerStateMachine : public FSM
    {
        public:
            PlannerStateMachine(rclcpp::Node::SharedPtr node, 
                std::shared_ptr<ROSActuatorInterface> actuator_interface, 
                std::shared_ptr<planner::Planner> planner) : 
                FSM(),
                _node(node),
                _actuator_interface(actuator_interface),
                _planner(planner)
            {
                _timer = _node->create_wall_timer(
                    std::chrono::milliseconds(10), 
                    std::bind(&PlannerStateMachine::_timer_callback, this));
            }

        public:
            /* state handlers */
            Status initial_state(const Event * const event) override;
            Status configuration_state(const Event *const event);
            Status planning_state(const Event *const event);
            Status finished_state(const Event *const event);

        private:
            rclcpp::Node::SharedPtr _node;
            std::shared_ptr<ROSActuatorInterface> _actuator_interface;
            std::shared_ptr<planner::Planner> _planner;
            rclcpp::TimerBase::SharedPtr _timer;

        private:
            void _timer_callback()
            {
                static uint8_t timer_counter = 0;

                static Event const timeout_10ms_event(static_cast<uint8_t>(Signal::TIMEOUT_10MS_SIG));
                event_manager::g_event_manager.post_event(&timeout_10ms_event); // 10ms timeout event

                if (timer_counter % 100 == 0) // 1s timeout event
                {
                    static Event const timeout_1s_event(static_cast<uint8_t>(Signal::TIMEOUT_1S_SIG));
                    event_manager::g_event_manager.post_event(&timeout_1s_event);
                }
                
                timer_counter++; // increment timer counter 
            }

    };
    
} // namespace planner_sm

#endif // STATEMACHINE_H