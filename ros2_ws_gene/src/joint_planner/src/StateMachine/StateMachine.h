
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <cstdint>
#include "rclcpp/rclcpp.hpp"

#include "../ActuatorInterface/ROSActuatorInterface.h"
#include "../Planner/Planner.h"
#include "../EventManager/EventManager.h"
#include "../FSM/FSM.h"


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
                _sampling_time = 0.005; // 5ms
                _timer = _node->create_wall_timer(
                    std::chrono::milliseconds(static_cast<int64_t>(_sampling_time * 1000)), 
                    std::bind(&PlannerStateMachine::_timer_callback, this));

                static event_manager::Event const entry_event(static_cast<uint8_t>(event_manager::Signal::ENTRY_SIG));
                (this->*_state)(&entry_event);
            }

        /* State handlers */
        public:
            Status initial_state(const Event * const event) override;
            Status configuration_state(const Event *const event);
            Status planning_state(const Event *const event);
            Status finished_state(const Event *const event);

        /* Composition objects */
        private:
            rclcpp::Node::SharedPtr _node;
            float _sampling_time;
            std::shared_ptr<ROSActuatorInterface> _actuator_interface;
            std::shared_ptr<planner::Planner> _planner;
        
        /* Timer related variables, methods */
        private:
            rclcpp::TimerBase::SharedPtr _timer;
            bool _is_started = false;
            uint16_t _timer_counter = 0;

        private:
            void _start_timer() { _is_started = true;}
            void _stop_timer() { _is_started = false; _timer_counter = 0; }
            void _timer_callback()
            {
                if (!_is_started) return;

                /* Start the timer */
                static Event const timeout_10ms_event(static_cast<uint8_t>(Signal::TIMEOUT_5MS_SIG));
                event_manager::g_event_manager.post_event(&timeout_10ms_event); // 10ms timeout event

                if ((_timer_counter == static_cast<uint8_t>(1.0 / _sampling_time))) // 1s timeout event
                {
                    static Event const timeout_1s_event(static_cast<uint8_t>(Signal::TIMEOUT_1S_SIG));
                    event_manager::g_event_manager.post_event(&timeout_1s_event);
                }
                else if ((_timer_counter == static_cast<uint8_t>(3.0 / _sampling_time))) // 3s timeout event
                {
                    static Event const timeout_3s_event(static_cast<uint8_t>(Signal::TIMEOUT_3S_SIG));
                    event_manager::g_event_manager.post_event(&timeout_3s_event);
                }

                _timer_counter++; // increment timer counter
                if (_timer_counter >= static_cast<uint8_t>(10.0 / _sampling_time)) // reset timer counter
                {
                    _timer_counter = 0;
                }
            }

    };
    
} // namespace planner_sm

#endif // STATEMACHINE_H