#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <fcntl.h>
#include <sys/mman.h>

#define SHM_NAME "/motor_state"
#define SHM_SIZE sizeof(MotorState)

struct MotorState
{
    double position[5];
    double angular_velocity[5];
    double torque[5];
    double elapsed_time; // ms
};

using MotorState = struct MotorState;

class MotorPublisher : public rclcpp::Node
{
public:
    MotorPublisher() : Node("motor_publisher")
    {
        std::cout << "MotorPublisher node started" << std::endl;
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/motor_state", 10);

        init_shared_memory();
        init_buffers();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            [this]()
            { this->publish_state(); });
    }

private:
    void init_buffers()
    {
        joint_state_.position.resize(5);
        joint_state_.velocity.resize(5);
        joint_state_.effort.resize(6);
    }

    void init_shared_memory()
    {
        shm_fd_ = shm_open(SHM_NAME, O_RDONLY, 0666);
        if (shm_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open shared memory");
            rclcpp::shutdown();
            return;
        }
        state_ = (MotorState *)mmap(NULL, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd_, 0);
    }

    void publish_state()
    {
        for (int i = 0; i < 5; ++i)
        {
            joint_state_.position[i] = state_->position[i];
            joint_state_.velocity[i] = state_->angular_velocity[i];
            joint_state_.effort[i] = state_->torque[i];
        }
        joint_state_.effort[5] = state_->elapsed_time; // ms
        joint_state_.header.stamp = this->now();
        publisher_->publish(joint_state_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    sensor_msgs::msg::JointState joint_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    int shm_fd_;
    MotorState *state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPublisher>());
    rclcpp::shutdown();
    return 0;
}