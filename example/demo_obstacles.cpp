#include <iostream>
#include <stdio.h>
#include <vector>
#include "bezier_tracking/bezier_curves.hpp"
#include "bezier_tracking/path_planner.hpp"
#include "bezier_tracking/obstacles.hpp"

int main(){
    // Map
    LinearBezier::Params line_params;

    line_params.x_start = 0;
    line_params.y_start = 0; 
    line_params.x_end   = 2;
    line_params.y_end   = 0;
    LinearBezier Line1(line_params);

    line_params.x_start = 2;
    line_params.y_start = 0; 
    line_params.x_end   = 2;
    line_params.y_end   = 2;
    LinearBezier Line2(line_params);

    Map map({&Line1,&Line2});

    // Obstacles
    Circle::Params circle_params;

    circle_params.xc = 1;
    circle_params.yc = 0; 
    circle_params.r  = 0.5;
    Circle circle1(circle_params);

    circle_params.xc = 2;
    circle_params.yc = 1; 
    circle_params.r  = 0.5;
    Circle circle2(circle_params);

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
    planner_params.obstacles = {&circle1,&circle2};
    planner_params.reactivity = 1;

    PathPlanner planner(planner_params);
    PathPlanner::Trajectory trajectory;

    // Simulation
    FILE* data_points;
    FILE *config_data,*map_data,*robot_data,*planner_data,*obstacles_data;
    int tsteps = 550;
    float x_current,y_current,theta_current;
    float vx,vy;
    float x_next,y_next,theta_next;

    float x0     = 0;
    float y0     = -0.1;
    float theta0 = 0;

    data_points    = fopen("data_points.tmp","w");
    config_data    = fopen("../simulation/config.txt","w");
    map_data       = fopen("../simulation/map.txt","w");
    robot_data     = fopen("../simulation/robot.txt","w");
    planner_data   = fopen("../simulation/planner.txt","w");
    obstacles_data = fopen("../simulation/obstacles.txt","w");

    // Config data
    fprintf(config_data,"%d\n",map.get_num_curves());
    fprintf(config_data,"%d\n",tsteps);
    fprintf(config_data,"%d\n",planner_params.horizon);
    fprintf(config_data,"%d\n",planner_params.obstacles.size());

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

    // Obstacles data
    for(auto obstacle : planner_params.obstacles){
        fprintf(obstacles_data, "%s\n",obstacle->shape().c_str());
        for (auto property : obstacle->properties()){
            fprintf(obstacles_data, "%f\n",property);
        }
    }

    // Simulation
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

        planner.obstacle_avoidance(x_current,y_current,theta_current,vx,vy);

        x_next = x_current + vx*planner.get_dt();
        y_next = y_current + vy*planner.get_dt();
        theta_next = planner.find_angle(1,0,vx,vy);

        x_current = x_next;
        y_current = y_next;
        theta_current = theta_next;

        fprintf(data_points, "%f %f\n",x_current,y_current);
        fprintf(robot_data,"%.2f,%.2f,%.2f\n",x_current,y_current,theta_current);
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
    
    return 0;
}