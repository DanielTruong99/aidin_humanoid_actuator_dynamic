#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <vector>


namespace planner 
{

    class Planner 
    {
        public:
            Planner();

            template <typename T>
            T compute(float dt);
            void reset();

        private:
            float _current_time;

            void _initialize();
    };

} // namespace Planner

#endif // PLANNER_H