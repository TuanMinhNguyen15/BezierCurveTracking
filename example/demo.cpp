#include <iostream>
#include <stdio.h>
#include <vector>
#include "bezier_tracking/linear_bezier.hpp"
#include "bezier_tracking/map.hpp"
#include "bezier_tracking/path_planner.hpp"

int main(){
    LinearBezier::Params line_params;

    // Line1
    line_params.x_start = 0;
    line_params.y_start = 0;
    line_params.x_end   = 1;
    line_params.y_end   = 0;
    LinearBezier Line1(line_params);

    // Line2
    line_params.x_start = 1;
    line_params.y_start = 0;
    line_params.x_end   = 1;
    line_params.y_end   = 1;
    LinearBezier Line2(line_params);

    // Line3
    line_params.x_start = 1;
    line_params.y_start = 1;
    line_params.x_end   = 2;
    line_params.y_end   = 1;
    LinearBezier Line3(line_params);

    // Create Map
    Map map({&Line1,&Line2,&Line3});

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
    FILE* data_points;
    FILE *config_data,*map_data,*robot_data,*planner_data;
    int tsteps = 300;
    float x_current,y_current,theta_current;
    float vx,vy;
    float x_next,y_next,theta_next;
    float x0     = 0;
    float y0     = 0;
    float theta0 = -2.35;


    data_points  = fopen("data_points.tmp","w");
    config_data  = fopen("../simulation/config.txt","w");
    map_data     = fopen("../simulation/map.txt","w");
    robot_data   = fopen("../simulation/robot.txt","w");
    planner_data = fopen("../simulation/planner.txt","w");

    // Config data
    fprintf(config_data,"%d\n",map.get_num_curves());
    fprintf(config_data,"%d\n",tsteps);
    fprintf(config_data,"%d\n",planner_params.horizon);

    // Map data
    auto curves = map.get_curves();
    std::vector<std::array<float,2>> control_points;
    for (auto curve : curves){
        if (curve->bezier_type() == "linear"){
            fprintf(map_data, "%s\n","linear");
            control_points = curve->get_control_points();
            for (auto control_point : control_points){
                fprintf(map_data, "%.2f,%.2f\n",control_point[0],control_point[1]);
            }
        }
    }

    
    x_current = x0;
    y_current = y0;
    theta_current = theta0;

    fprintf(data_points, "%f %f\n",x_current,y_current);
    fprintf(robot_data,"%.2f,%.2f,%.2f\n",x_current,y_current,theta_current);

    planner.priming(x_current,y_current,theta_current);
    for(int i = 0; i < tsteps; i++){
        trajectory = planner.tracking(x_current,y_current,theta_current);
        // Planner data
        for (auto pos : trajectory.pos){
            fprintf(planner_data,"%.2f,%.2f\n",pos[0],pos[1]);
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
        fprintf(robot_data,"%.2f,%.2f,%.2f\n",x_current,y_current,theta_current);
        fprintf(data_points, "%f %f\n",x_current,y_current);
    }
    trajectory = planner.tracking(x_current,y_current,theta_current);
    // Planner data
    for (auto pos : trajectory.pos){
        fprintf(planner_data,"%.2f,%.2f\n",pos[0],pos[1]);
    }


    fclose(data_points);
    fclose(config_data);
    fclose(map_data);
    fclose(robot_data);
    fclose(planner_data);
    fclose(data_points);


    // std::cout << "angle = " << planner.find_angle(-1,1,-1,-1) << std::endl;
    


    // trajectory = planner.tracking(x_current,y_current,theta_current);
    // for (auto pos : trajectory.pos){
    //     // std::cout << "x = " << pos[0] << " , y = " << pos[1] << std::endl;
    //     fprintf(data_points, "%f %f\n",pos[0],pos[1]);
    // }



    // float x,y;
    // planner.params_.map->get_velocity(.5,1,x,y);
    // std::cout << x << " , " << y << std::endl;
}