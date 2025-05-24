#include <fcntl.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/sem.h>
#include <alchemy/mutex.h>
#include <alchemy/timer.h>
#include <trank/rtdk.h>
#include <rtdm/ipc.h>

#include <math.h>

struct MotorState
{
    double position;         // Radians
    double angular_velocity; // Rad/s
    double torque;           // Nm
};

using MotorState = struct MotorState;

#define SHM_NAME "/motor_state"
#define SHM_SIZE sizeof(MotorState)

void rt_task_proc(void *arg)
{
    MotorState *state = (MotorState *)arg;
    unsigned long loop_cnt = 0;

    rt_task_set_periodic(NULL, TM_NOW, 1e6); // 1ms period

    while (1)
    {
        // Update motor state (simulated values)
        state->position = loop_cnt * 0.001;
        state->angular_velocity = 10.0 * std::sin(loop_cnt * 0.01);
        state->torque = 5.0 * std::cos(loop_cnt * 0.005);

        loop_cnt++;
        rt_task_wait_period(NULL);
    }
}

int main(int argc, char **argv)
{
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, SHM_SIZE);

    MotorState *state = (MotorState *)mmap(NULL, SHM_SIZE,
                                           PROT_READ | PROT_WRITE,
                                           MAP_SHARED, shm_fd, 0);
    mlock(state, SHM_SIZE); // Lock memory to prevent page faults

    RT_TASK rt_task;
    rt_task_create(&rt_task, "RTWriter", 0, 80, T_JOINABLE);
    rt_task_start(&rt_task, rt_task_proc, state);
    rt_task_join(&rt_task);

    munmap(state, SHM_SIZE);
    close(shm_fd);
    shm_unlink(SHM_NAME);
    return 0;
}