#include "bezier_tracking/path_planner.hpp"

PathPlanner::PathPlanner(Params params):params_(params){
    theta_diff_max = std::atan(params_.vel*params_.dt/params_.r_min);
}

void PathPlanner::update_map(Map *map){
    params_.map = map;
}

void PathPlanner::update_lambda(float lambda){
    this->lambda = lambda;
}

float PathPlanner::get_dt(){
    return params_.dt;
}

float PathPlanner::find_angle(float x1, float y1, float x2, float y2){
    double dot;
    double mag1,mag2;
    double arg;
    float angle;

    dot = (x1*x2) + (y1*y2);
    mag1 = std::sqrt(std::pow(x1,2) + std::pow(y1,2));
    mag2 = std::sqrt(std::pow(x2,2) + std::pow(y2,2));
    arg = dot/(mag1*mag2);
    arg = std::min(std::max(arg,-1.),1.);

    angle = std::acos(arg);
    if (x1*y2 - y1*x2 < 0){
        angle *= -1;
    }

    return angle;
}

void PathPlanner::rotate(float &x, float &y, float angle){
    float x_temp = x;
    float y_temp = y;

    x = std::cos(angle)*x_temp - std::sin(angle)*y_temp;
    y = std::sin(angle)*x_temp + std::cos(angle)*y_temp;
}

PathPlanner::Trajectory PathPlanner::pursuitcurve_sim(float x0, float y0, float theta0, float lambda){
    float x_lead,y_lead;
    float x_follow, y_follow;
    float vx_lead,vy_lead;
    float vx_follow,vy_follow;
    float heading_x,heading_y;
    float distance,angle,angle_diff;
    float theta_current,theta_next,theta_diff;
    float lambda_vel;
    Trajectory trajectory;

    trajectory.pos.clear();
    trajectory.vel.clear();
    trajectory.pos.resize(params_.horizon+1);
    trajectory.vel.resize(params_.horizon+1);
    trajectory.total_cost = 0;

    x_follow = x0;
    y_follow = y0;
    heading_x = 1;
    heading_y = 0;
    rotate(heading_x,heading_y,theta0);
    // theta_current = theta0;
    for (int i = 0; i <= params_.horizon; i++){
        // Find v_follow
        params_.map->get_point(lambda,x_lead,y_lead);
        distance = std::sqrt(std::pow(x_lead-x_follow,2) + std::pow(y_lead-y_follow,2));
        lambda_vel = params_.map->lambda_velocity(lambda,params_.vel);
        params_.map->get_velocity(lambda,lambda_vel,vx_lead,vy_lead);
        if (distance < 1e-4){
            vx_follow = vx_lead;
            vy_follow = vy_lead;
            angle = 0;
        }
        else{
            vx_follow = params_.vel*(x_lead-x_follow)/distance;
            vy_follow = params_.vel*(y_lead-y_follow)/distance;
            angle = find_angle(vx_lead,vy_lead,vx_follow,vy_follow);

        }

        // Obstacle testing
        // obstacle_avoidance(x0,y0,theta0,vx_follow,vy_follow);

        // Constrain v_follow w.r.t minimum turning radius
        theta_diff = find_angle(vx_follow,vy_follow,heading_x,heading_y);
        if(std::abs(theta_diff) > theta_diff_max){
            if(theta_diff <= 0){
                // Update v_follow
                rotate(vx_follow,vy_follow,theta_diff+theta_diff_max);
            }
            else{
                // Update v_follow
                rotate(vx_follow,vy_follow,theta_diff-theta_diff_max);
            }
            angle_diff = theta_diff_max;
            // theta_current = find_angle(1,0,vx_follow,vy_follow);
        }
        else{
            angle_diff = std::abs(theta_diff);
            // theta_current = theta_next;
        }
        heading_x = vx_follow;
        heading_y = vy_follow;
        
        // Calculate total cost
        trajectory.total_cost += params_.distance_cost*distance;
        trajectory.total_cost += params_.heading_cost*std::abs(angle);
        trajectory.total_cost += params_.smooth_cost*angle_diff;

        // Store results
        trajectory.pos[i][0] = x_follow;
        trajectory.pos[i][1] = y_follow;
        trajectory.vel[i][0] = vx_follow;
        trajectory.vel[i][1] = vy_follow;

        // Update next step
        x_follow += vx_follow*params_.dt;
        y_follow += vy_follow*params_.dt;
        lambda   += lambda_vel*params_.dt;
    }

    return trajectory;
}

PathPlanner::Trajectory PathPlanner::tracking(float x0, float y0, float theta0){
    Trajectory traj_forward,traj_backward;
    Trajectory traj_best,traj_test;
    float lambda_diff = params_.map->get_lambda_diff(lambda);
    
    traj_best     = pursuitcurve_sim(x0,y0,theta0,lambda);
    traj_forward  = pursuitcurve_sim(x0,y0,theta0,lambda+lambda_diff);
    traj_backward = pursuitcurve_sim(x0,y0,theta0,lambda-lambda_diff);

    if (traj_forward.total_cost <= traj_backward.total_cost){
        // Favor forward
        while(true){
            if (traj_best.total_cost <= traj_forward.total_cost){
                // std::cout << "lambda_best = " << lambda << std::endl;
                return traj_best;
            }
            else{
                traj_best    = traj_forward;
                lambda      += lambda_diff;
                lambda_diff = params_.map->get_lambda_diff(lambda);
                traj_forward = pursuitcurve_sim(x0,y0,theta0,lambda+lambda_diff);
            }
        }
    }
    else{
        // Favor backward
        while(true){
            lambda_diff = params_.map->get_lambda_diff(lambda);
            if (traj_best.total_cost <= traj_backward.total_cost){
                // std::cout << "lambda_best = " << lambda << std::endl;
                return traj_best;
            }
            else{
                traj_best     = traj_backward;
                lambda       -= lambda_diff;
                lambda_diff = params_.map->get_lambda_diff(lambda);
                traj_backward = pursuitcurve_sim(x0,y0,theta0,lambda-lambda_diff);
            }
        }
    }

}

void PathPlanner::priming(float x0, float y0, float theta0){
    Trajectory trajectory;
    float cost_min;
    float lambda_best;
    int num_curves = params_.map->get_num_curves();

    lambda = 0;
    trajectory = tracking(x0,y0,theta0);
    cost_min = trajectory.total_cost;
    lambda_best = lambda;

    for (float lambda_test = 1;lambda_test < num_curves;lambda_test+=1){
        lambda = lambda_test;
        trajectory = tracking(x0,y0,theta0);
        if (trajectory.total_cost < cost_min){
            lambda_best = lambda;
        }
    }

    lambda = lambda_best;
    std::cout << "lambda_best = " << lambda << std::endl;
} 

void PathPlanner::obstacle_avoidance(float x0, float y0, float theta0, float &vx, float &vy){
    float nx,ny;
    float ex,ey;
    float det;
    float v_perp,v_away;
    float v_mag;

    float vx_avg,vy_avg;
    std::vector<float> vx_list,vy_list;
    std::vector<float> eval_list;

    float heading_x = 1;
    float heading_y = 0;
    float theta_diff;

    for (auto obstacle : params_.obstacles){
        eval_list.push_back(obstacle->evaluate(x0,y0));

        obstacle->gradient(x0,y0,nx,ny);
        ex =  ny;
        ey = -nx;

        det = nx*ey - ex*ny;
        v_perp =  (ey/det)*vx - (ex/det)*vy;
        v_away = -(ny/det)*vx + (nx/det)*vy;

        // Modulation
        if (nx*vx + ny*vy < 0 || obstacle->evaluate(x0,y0) <= 1){
            v_perp = v_perp*(1 - 1/std::pow(obstacle->evaluate(x0,y0),1/params_.reactivity));
            v_perp = std::min(std::max(v_perp,-params_.vel),params_.vel);
            if (v_away >= 0){
                v_away = std::sqrt(std::pow(params_.vel,2) - std::pow(v_perp,2));
            }
            else{
                v_away = -std::sqrt(std::pow(params_.vel,2) - std::pow(v_perp,2));
            }
        }

        vx_list.push_back(nx*v_perp + ex*v_away);
        vy_list.push_back(ny*v_perp + ey*v_away);
    }

    // Calculate average
    std::vector<float> w;
    w.resize(params_.obstacles.size());
    float w_total = 0;

    for (int i = 0; i < params_.obstacles.size(); i++){
        w[i] = 1;
        for (int j = 0; j < params_.obstacles.size(); j++){
            if (j != i){
                w[i] *= (eval_list[j]-1);
            }
        }
        w_total += w[i];
    }

    vx = 0;
    vy = 0;
    for(int k = 0; k < params_.obstacles.size(); k++){
        vx += (w[k]/w_total)*vx_list[k];
        vy += (w[k]/w_total)*vy_list[k];
    }


    rotate(heading_x,heading_y,theta0);
    theta_diff = find_angle(vx,vy,heading_x,heading_y);
    if(std::abs(theta_diff) > theta_diff_max){
        if(theta_diff <= 0){
            // Update v_follow
            rotate(vx,vy,theta_diff+theta_diff_max);
        }
        else{
            // Update v_follow
            rotate(vx,vy,theta_diff-theta_diff_max);
        }
    }

}