#include <iostream>
#include <stdio.h>
#include <vector>
#include "bezier_tracking/bezier_curves.hpp"
#include "bezier_tracking/path_planner.hpp"

int main(){
    QuadraticBezier::Params quadratic_params;

    quadratic_params.x_start = 0;
    quadratic_params.y_start = 0;
    quadratic_params.x_inter = 1;
    quadratic_params.y_inter = 2;
    quadratic_params.x_end   = 2;
    quadratic_params.y_end   = 0;
    quadratic_params.dstep   = 0.01;
    QuadraticBezier Quad(quadratic_params);

    // Create Map
    Map map({&Quad});


    // Setup Path Planner
    PathPlanner::Params planner_params;
    planner_params.map = &map;
    planner_params.vel = 1;
    planner_params.horizon = 150;
    planner_params.r_min = 0.1;
    planner_params.dt = 0.01;
    planner_params.distance_cost = 1;
    planner_params.heading_cost  = 5;
    planner_params.smooth_cost   = 2;

    PathPlanner planner(planner_params);
    PathPlanner::Trajectory trajectory;


    // Simulation
    FILE *general;
    FILE* data_points1;
    FILE *config_data1,*map_data1,*robot_data1,*planner_data1,*obstacles_data;
    int tsteps = 400;
    float x_current,y_current,theta_current;
    float vx,vy;
    float x_next,y_next,theta_next;
    float x0     = -0.5;
    float y0     = 1;
    float theta0 = -3;


    general         = fopen("../simulation/general.txt","w");
    data_points1    = fopen("data_points1.tmp","w");
    config_data1    = fopen("../simulation/config1.txt","w");
    map_data1       = fopen("../simulation/map1.txt","w");
    robot_data1     = fopen("../simulation/robot1.txt","w");
    planner_data1   = fopen("../simulation/planner1.txt","w");
    obstacles_data  = fopen("../simulation/obstacles.txt","w");


    // General data
    fprintf(general,"%d\n",tsteps);  // number of timesteps
    fprintf(general,"%d\n",1); // number of agents
    fprintf(general,"%d\n",0); // number of obstacles


    // Config data
    fprintf(config_data1,"%d\n",map.get_num_curves());    // number of map curves
    fprintf(config_data1,"%d\n",planner_params.horizon);  // planner horizon

    // Map data
    auto curves = map.get_curves();
    std::vector<std::array<float,2>> control_points;
    for (auto curve : curves){
        fprintf(map_data1, "%s\n",curve->bezier_type().c_str());
            control_points = curve->get_control_points();
            for (auto control_point : control_points){
                fprintf(map_data1, "%.2f,%.2f\n",control_point[0],control_point[1]);
            }
    }

    
    x_current = x0;
    y_current = y0;
    theta_current = theta0;

    fprintf(data_points1, "%f %f\n",x_current,y_current);
    fprintf(robot_data1,"%.2f,%.2f,%.2f\n",x_current,y_current,theta_current);

    planner.priming(x_current,y_current,theta_current);
    for(int i = 0; i < tsteps; i++){
        trajectory = planner.tracking(x_current,y_current,theta_current);
        // Planner data
        for (auto pos : trajectory.pos){
            fprintf(planner_data1,"%.2f,%.2f\n",pos[0],pos[1]);
        }

        // Robot data
        vx = trajectory.vel[0][0];
        vy = trajectory.vel[0][1];
        x_next = x_current + vx*planner.get_dt();
        y_next = y_current + vy*planner.get_dt();
        theta_next = planner.find_angle(1,0,vx,vy);

        x_current = x_next;
        y_current = y_next;
        theta_current = theta_next;
        fprintf(robot_data1,"%.2f,%.2f,%.2f\n",x_current,y_current,theta_current);
        fprintf(data_points1, "%f %f\n",x_current,y_current);
    }
    trajectory = planner.tracking(x_current,y_current,theta_current);
    // Planner data
    for (auto pos : trajectory.pos){
        fprintf(planner_data1,"%.2f,%.2f\n",pos[0],pos[1]);
    }


    fclose(data_points1);
    fclose(config_data1);
    fclose(map_data1);
    fclose(robot_data1);
    fclose(planner_data1);
    fclose(data_points1);
    fclose(obstacles_data);
}