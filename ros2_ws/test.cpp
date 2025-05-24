#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>
#include "state_machine.hpp"
#include "robot_controller.hpp"
#include "robot.hpp"

class RobotEvent : public state_machine::BuiltInEvent {
public:
    static constexpr int TIME_OUT_2S = state_machine::BuiltInEvent::USER_SIG;
    static constexpr int TIMER_EVENT = state_machine::BuiltInEvent::USER_SIG + 1;
    static constexpr int ABNORMAL_STATE_DETECTED = state_machine::BuiltInEvent::USER_SIG + 2;
    static constexpr int BACK_BUTTON_PRESSED = state_machine::BuiltInEvent::USER_SIG + 3;
    static constexpr int START_BUTTON_3S = state_machine::BuiltInEvent::USER_SIG + 4;
};

class RobotFSM : public state_machine::FSM {
public:
    RobotFSM(std::shared_ptr<Robot> robot, std::shared_ptr<RobotController> controller)
        : robot_(robot), controller_(controller) {}

    state_machine::Status initial_state(const RobotEvent& event) {
        state_machine::Status status = state_machine::Status::IGNORED_STATUS;

        if (event == RobotEvent::ENTRY_SIG) {
            RCLCPP_INFO(rclcpp::get_logger("RobotFSM"), "Robot in initial state");
            status = state_machine::Status::HANDLED_STATUS;
        } else if (event == RobotEvent::TIMER_EVENT) {
            if (robot_->is_ready()) {
                RCLCPP_INFO(rclcpp::get_logger("RobotFSM"), "All sensor data are ready");
                transition_to(&RobotFSM::configuration_state);
                status = state_machine::Status::TRAN_STATUS;
            }
        }

        return status;
    }

    state_machine::Status configuration_state(const RobotEvent& event) {
        state_machine::Status status = state_machine::Status::IGNORED_STATUS;

        if (event == RobotEvent::ENTRY_SIG) {
            controller_->start_moving_to_default(2.0);
            RCLCPP_INFO(rclcpp::get_logger("RobotFSM"), "Robot in configuration state");
            status = state_machine::Status::HANDLED_STATUS;
        } else if (event == RobotEvent::TIMER_EVENT) {
            if (controller_->is_start_moving_to_default()) {
                controller_->move_to_default_position("kneel");

                if (controller_->is_done_moving_to_default_position()) {
                    RCLCPP_INFO(rclcpp::get_logger("RobotFSM"), "Robot is done moving to kneeling position");
                    RCLCPP_INFO(rclcpp::get_logger("RobotFSM"), "Robot is waiting for 2 seconds...");
                    controller_->reset_done_moving_flag();
                    controller_->start_timer();
                    status = state_machine::Status::HANDLED_STATUS;
                }
            }
        } else if (event == RobotEvent::TIME_OUT_2S) {
            controller_->reset_timer();
            transition_to(&RobotFSM::running_state);
            status = state_machine::Status::TRAN_STATUS;
        }

        return status;
    }

    state_machine::Status running_state(const RobotEvent& event) {
        state_machine::Status status = state_machine::Status::IGNORED_STATUS;

        if (event == RobotEvent::TIMER_EVENT) {
            auto joint_cmds = controller_->compute();
            auto joint_cmd_msg = sensor_msgs::msg::JointState();
            joint_cmd_msg.header.stamp = controller_->get_clock()->now();
            joint_cmd_msg.position = joint_cmds;
            joint_cmd_msg.velocity = controller_->get_kps_and_kds();
            controller_->publish_joint_cmd(joint_cmd_msg);

            status = state_machine::Status::HANDLED_STATUS;
        } else if (event == RobotEvent::ENTRY_SIG) {
            RCLCPP_INFO(rclcpp::get_logger("RobotFSM"), "Robot in running state");
            status = state_machine::Status::HANDLED_STATUS;
        } else if (event == RobotEvent::ABNORMAL_STATE_DETECTED) {
            transition_to(&RobotFSM::error_state);
            status = state_machine::Status::TRAN_STATUS;
        } else {
            if (!robot_->is_safe()) {
                RCLCPP_ERROR(rclcpp::get_logger("RobotFSM"), "Abnormal state detected!");
                controller_->push_event(RobotEvent::ABNORMAL_STATE_DETECTED);
                status = state_machine::Status::HANDLED_STATUS;
            }
        }

        return status;
    }

    state_machine::Status error_state(const RobotEvent& event) {
        state_machine::Status status = state_machine::Status::IGNORED_STATUS;

        if (event == RobotEvent::ENTRY_SIG) {
            controller_->stop();
            controller_->reset();
            RCLCPP_ERROR(rclcpp::get_logger("RobotFSM"), "Robot in error state");
            status = state_machine::Status::HANDLED_STATUS;
        } else if (event == RobotEvent::BACK_BUTTON_PRESSED) {
            controller_->start_moving_to_default(5.0);
            status = state_machine::Status::HANDLED_STATUS;
        } else if (event == RobotEvent::TIMER_EVENT) {
            if (controller_->is_start_moving_to_default()) {
                controller_->move_to_default_position("stand");

                if (controller_->is_done_moving_to_default_position()) {
                    RCLCPP_INFO(rclcpp::get_logger("RobotFSM"), "Robot is done moving to standing position");
                    status = state_machine::Status::HANDLED_STATUS;
                }
            }
        } else if (event == RobotEvent::START_BUTTON_3S) {
            transition_to(&RobotFSM::initial_state);
            status = state_machine::Status::TRAN_STATUS;
        }

        return status;
    }

private:
    std::shared_ptr<Robot> robot_;
    std::shared_ptr<RobotController> controller_;
};