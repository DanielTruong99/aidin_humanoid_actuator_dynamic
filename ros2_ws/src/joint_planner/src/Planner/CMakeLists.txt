add_library(planner STATIC Planner.h Planner.cpp)

target_include_directories(planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
