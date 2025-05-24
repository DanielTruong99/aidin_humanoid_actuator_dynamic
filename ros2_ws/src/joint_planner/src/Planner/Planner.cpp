#include "Planner.h"

namespace planner
{
    Planner::Planner()
    {
        _initialize();
    }

    void Planner::_initialize()
    {
        _current_time = 0.0f;
    }

    template <typename T>
    T Planner::compute(float dt)
    {
        _current_time += dt;
        T joint_positions;
        return joint_positions;
    }

    void Planner::reset()
    {
        _initialize();
    }
} // namespace planner