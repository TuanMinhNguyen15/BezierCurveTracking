cmake_minimum_required(VERSION 3.24)
project(PursuitCurve)
 
add_library(line_tracking src/line_tracking.cpp)
target_include_directories(line_tracking PUBLIC include)

add_library(bezier_curve_tracking 
            src/bezier_curve.cpp 
            src/linear_bezier.cpp 
            src/map.cpp
            src/path_planner.cpp)
target_include_directories(bezier_curve_tracking PUBLIC include)

add_executable(demo example/demo.cpp)
target_link_libraries(demo bezier_curve_tracking)


