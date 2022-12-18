#include <iostream>
#include <stdio.h>
#include <vector>
#include "bezier_tracking/bezier_curves.hpp"
#include "bezier_tracking/path_planner.hpp"
#include "bezier_tracking/obstacles.hpp"

int main(){
    LinearBezier::Params line_params;

    line_params.x_start = 0; 
    line_params.y_start = 0;
    line_params.x_end   = 2;
    line_params.y_end   = 0;
    LinearBezier Line_robot1(line_params);

    line_params.x_start = 2; 
    line_params.y_start = 0;
    line_params.x_end   = 0;
    line_params.y_end   = 0;
    LinearBezier Line_robot2(line_params);

    Map map_robot1({&Line_robot1});
    Map map_robot2({&Line_robot2});

    Circle obstacle_robot1;
    obstacle_robot1.update_radius(0.1);
    Circle obstacle_robot2;
    obstacle_robot2.update_radius(0.1);


    PathPlanner::Params robot1_params;
    robot1_params.map = &map_robot1;
    robot1_params.vel = 1;
    robot1_params.horizon = 150;
    robot1_params.r_min = 0.1;
    robot1_params.dt = 0.01;
    robot1_params.distance_cost = 1;
    robot1_params.heading_cost  = 5;
    robot1_params.smooth_cost   = 2;
    robot1_params.obstacles = {&obstacle_robot2};
    PathPlanner robot1(robot1_params);


    PathPlanner::Params robot2_params;
    robot2_params.map = &map_robot2;
    robot2_params.vel = 1;
    robot2_params.horizon = 150;
    robot2_params.r_min = 0.1;
    robot2_params.dt = 0.01;
    robot2_params.distance_cost = 1;
    robot2_params.heading_cost  = 5;
    robot2_params.smooth_cost   = 2;
    robot2_params.obstacles = {&obstacle_robot1};
    PathPlanner robot2(robot2_params);

    PathPlanner::Trajectory robot1_traj,robot2_traj;


    // Simulation
    FILE *general;
    FILE *config_data1,*config_data2;
    FILE *data_points1,*data_points2;
    FILE *map_data1,*robot_data1,*planner_data1;
    FILE *map_data2,*robot_data2,*planner_data2;
    FILE *obstacles_data;

    int tsteps = 210;

    float robot1_x_current,robot1_y_current,robot1_theta_current;
    float robot1_vx,robot1_vy;
    float robot1_x_next,robot1_y_next,robot1_theta_next;

    float robot2_x_current,robot2_y_current,robot2_theta_current;
    float robot2_vx,robot2_vy;
    float robot2_x_next,robot2_y_next,robot2_theta_next;

    float robot1_x0     = 0;
    float robot1_y0     = 0;
    float robot1_theta0 = 1.57;

    float robot2_x0     = 2;
    float robot2_y0     = 0;
    float robot2_theta0 = -1.57;

    general         = fopen("../simulation/general.txt","w");

    config_data1    = fopen("../simulation/config1.txt","w");
    config_data2    = fopen("../simulation/config2.txt","w");

    data_points1    = fopen("data_points1.tmp","w");
    data_points2    = fopen("data_points2.tmp","w");

    map_data1       = fopen("../simulation/map1.txt","w");
    map_data2       = fopen("../simulation/map2.txt","w");

    robot_data1     = fopen("../simulation/robot1.txt","w");
    robot_data2     = fopen("../simulation/robot2.txt","w");

    planner_data1   = fopen("../simulation/planner1.txt","w");
    planner_data2   = fopen("../simulation/planner2.txt","w");

    obstacles_data  = fopen("../simulation/obstacles.txt","w");

    // General data
    fprintf(general,"%d\n",tsteps);  // number of timesteps
    fprintf(general,"%d\n",2); // number of agents
    fprintf(general,"%d\n",0); // number of obstacles

    // Config data
    fprintf(config_data1,"%d\n",map_robot1.get_num_curves()); // number of map curves
    fprintf(config_data1,"%d\n",robot1_params.horizon);       // planner horizon

    fprintf(config_data2,"%d\n",map_robot2.get_num_curves()); // number of map curves
    fprintf(config_data2,"%d\n",robot2_params.horizon);       // planner horizon

    // Map data
    auto curves = map_robot1.get_curves();
    std::vector<std::array<float,2>> control_points;
    for (auto curve : curves){
        if (curve->bezier_type() == "linear"){
            fprintf(map_data1, "%s\n","linear");
            control_points = curve->get_control_points();
            for (auto control_point : control_points){
                fprintf(map_data1, "%.2f,%.2f\n",control_point[0],control_point[1]);
            }
        }
    }

    curves = map_robot2.get_curves();
    for (auto curve : curves){
        if (curve->bezier_type() == "linear"){
            fprintf(map_data2, "%s\n","linear");
            control_points = curve->get_control_points();
            for (auto control_point : control_points){
                fprintf(map_data2, "%.2f,%.2f\n",control_point[0],control_point[1]);
            }
        }
    }

    // Obstacles data
    // for(auto obstacle : planner_params.obstacles){
    //     fprintf(obstacles_data, "%s\n",obstacle->shape().c_str());
    //     for (auto property : obstacle->properties()){
    //         fprintf(obstacles_data, "%f\n",property);
    //     }
    // }


    robot1_x_current = robot1_x0;
    robot1_y_current = robot1_y0;
    robot1_theta_current = robot1_theta0;

    robot2_x_current = robot2_x0;
    robot2_y_current = robot2_y0;
    robot2_theta_current = robot2_theta0;

    fprintf(data_points1, "%f %f\n",robot1_x_current,robot1_y_current);
    fprintf(data_points2, "%f %f\n",robot2_x_current,robot2_y_current);

    fprintf(robot_data1,"%.2f,%.2f,%.2f\n",robot1_x_current,robot1_y_current,robot1_theta_current);
    fprintf(robot_data2,"%.2f,%.2f,%.2f\n",robot2_x_current,robot2_y_current,robot2_theta_current);

    robot1.priming(robot1_x_current,robot1_y_current,robot1_theta_current);
    robot2.priming(robot2_x_current,robot2_y_current,robot2_theta_current);
    for(int i = 0; i < tsteps; i++){
        obstacle_robot1.update_center(robot1_x_current,robot1_y_current);
        obstacle_robot2.update_center(robot2_x_current,robot2_y_current);

        robot1_traj = robot1.tracking(robot1_x_current,robot1_y_current,robot1_theta_current);
        robot2_traj = robot2.tracking(robot2_x_current,robot2_y_current,robot2_theta_current);

        // Planner data
        for (auto pos : robot1_traj.pos){
            fprintf(planner_data1,"%.2f,%.2f\n",pos[0],pos[1]);
        }
        for (auto pos : robot2_traj.pos){
            fprintf(planner_data2,"%.2f,%.2f\n",pos[0],pos[1]);
        }

        // Robot data
        robot1_vx = robot1_traj.vel[0][0];
        robot1_vy = robot1_traj.vel[0][1];
        robot2_vx = robot2_traj.vel[0][0];
        robot2_vy = robot2_traj.vel[0][1];

        robot1.obstacle_avoidance(robot1_x_current,robot1_y_current,robot1_theta_current,robot1_vx,robot1_vy);
        robot2.obstacle_avoidance(robot2_x_current,robot2_y_current,robot2_theta_current,robot2_vx,robot2_vy);

        robot1_x_next = robot1_x_current + robot1_vx*robot1.get_dt();
        robot1_y_next = robot1_y_current + robot1_vy*robot1.get_dt();
        robot1_theta_next = robot1.find_angle(1,0,robot1_vx,robot1_vy);

        robot2_x_next = robot2_x_current + robot2_vx*robot2.get_dt();
        robot2_y_next = robot2_y_current + robot2_vy*robot2.get_dt();
        robot2_theta_next = robot2.find_angle(1,0,robot2_vx,robot2_vy);

        robot1_x_current = robot1_x_next;
        robot1_y_current = robot1_y_next;
        robot1_theta_current = robot1_theta_next;

        robot2_x_current = robot2_x_next;
        robot2_y_current = robot2_y_next;
        robot2_theta_current = robot2_theta_next;

        fprintf(data_points1, "%f %f\n",robot1_x_current,robot1_y_current);
        fprintf(data_points2, "%f %f\n",robot2_x_current,robot2_y_current);

        fprintf(robot_data1,"%.2f,%.2f,%.2f\n",robot1_x_current,robot1_y_current,robot1_theta_current);
        fprintf(robot_data2,"%.2f,%.2f,%.2f\n",robot2_x_current,robot2_y_current,robot2_theta_current);
    }
    robot1_traj = robot1.tracking(robot1_x_current,robot1_y_current,robot1_theta_current);
    robot2_traj = robot2.tracking(robot2_x_current,robot2_y_current,robot2_theta_current);

    // Planner data
    for (auto pos : robot1_traj.pos){
        fprintf(planner_data1,"%.2f,%.2f\n",pos[0],pos[1]);
    }
    for (auto pos : robot2_traj.pos){
        fprintf(planner_data2,"%.2f,%.2f\n",pos[0],pos[1]);
    }

    fclose(general);

    fclose(data_points1);
    fclose(data_points2);

    fclose(config_data1);
    fclose(config_data2);

    fclose(map_data1);
    fclose(map_data2);

    fclose(robot_data1);
    fclose(robot_data2);

    fclose(planner_data1);
    fclose(planner_data2);

    fclose(obstacles_data);
    
    return 0;
}

