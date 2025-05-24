#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

#define SHM_NAME "/motor_command"
#define SHM_SIZE sizeof(MotorCommand)

struct MotorCommand
{
    double position[5];
    double velocity[5];
    double effort[5];
};

using MotorCommand = struct MotorCommand;

class MotorCommandSubscriber : public rclcpp::Node
{
public:
    MotorCommandSubscriber() : Node("motor_command_subscriber")
    {
        std::cout << "MotorCommandSubscriber node started" << std::endl;
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/filtered_joint_cmd", 10,
            std::bind(&MotorCommandSubscriber::command_callback, this, std::placeholders::_1));

        init_shared_memory();
    }

    ~MotorCommandSubscriber()
    {
        if (state_) munmap(state_, SHM_SIZE);
        if (shm_fd_ >= 0) close(shm_fd_);
    }

private:
    void init_shared_memory()
    {
        shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (shm_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open shared memory");
            rclcpp::shutdown();
            return;
        }
        ftruncate(shm_fd_, SHM_SIZE);
        state_ = (MotorCommand *)mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
        if (state_ == MAP_FAILED)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to mmap shared memory");
            rclcpp::shutdown();
        }
        std::memset(state_, 0, SHM_SIZE);
    }

    void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        /* Copy data to shared memory */
        for (size_t index = 0; index < 5; index++)
        {
            /* Ensure we don't access out of bounds */
            state_->position[index] = (index < msg->position.size()) ? msg->position[index] : 0.0;
            state_->velocity[index] = (index < msg->velocity.size()) ? msg->velocity[index] : 0.0;
            state_->effort[index] = (index < msg->effort.size()) ? msg->effort[index] : 0.0;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    int shm_fd_{-1};
    MotorCommand *state_{nullptr};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorCommandSubscriber>());
    rclcpp::shutdown();
    return 0;
}
