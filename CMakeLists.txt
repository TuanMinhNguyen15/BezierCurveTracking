cmake_minimum_required(VERSION 3.24)
project(PursuitCurve)

add_library(bezier_curve_tracking 
            src/bezier_curves.cpp 
            src/path_planner.cpp
            src/obstacles.cpp)
target_include_directories(bezier_curve_tracking PUBLIC include)

add_executable(demo_tracking example/demo_tracking.cpp)
target_link_libraries(demo_tracking bezier_curve_tracking)

add_executable(demo_obstacles example/demo_obstacles.cpp)
target_link_libraries(demo_obstacles bezier_curve_tracking)

add_executable(demo_multi_agents example/demo_multi_agents.cpp)
target_link_libraries(demo_multi_agents bezier_curve_tracking)


