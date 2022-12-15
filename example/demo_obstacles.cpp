#include <iostream>
#include <stdio.h>
#include <vector>
#include "bezier_tracking/linear_bezier.hpp"
#include "bezier_tracking/map.hpp"
#include "bezier_tracking/path_planner.hpp"
#include "bezier_tracking/obstacles.hpp"

int main(){
    Circle::Params circle_params;

    circle_params.xc = 0;
    circle_params.yc = 0; 
    circle_params.r  = 1;

    Circle circle(circle_params);

   // Setup Path Planner
    PathPlanner::Params planner_params;
    // planner_params.map = &map;
    planner_params.vel = 1;
    planner_params.horizon = 150;
    planner_params.r_min = 0.1;
    planner_params.dt = 0.01;
    planner_params.distance_cost = 1;
    planner_params.heading_cost  = 5;
    planner_params.smooth_cost   = 2;
    planner_params.obstacle = &circle;

    PathPlanner planner(planner_params);

    FILE* data_points;
    int tsteps = 1000;
    float x_current,y_current;
    float vx,vy;
    float x_next,y_next;

    float x0     = -2;
    float y0     = -0.1;


    float x_target = 5;
    float y_target = 0;

    data_points  = fopen("data_points.tmp","w");

    x_current = x0;
    y_current = y0;
    fprintf(data_points, "%f %f\n",x_current,y_current);
    for(int i = 0; i < tsteps; i++){
        vx = x_target - x_current;
        vy = y_target - y_current;

        planner.obstacle_avoidance(x_current,y_current,0,vx,vy);

        x_next = x_current + vx*planner.get_dt();
        y_next = y_current + vy*planner.get_dt();

        x_current = x_next;
        y_current = y_next;

        fprintf(data_points, "%f %f\n",x_current,y_current);
    }
    
    return 0;
}