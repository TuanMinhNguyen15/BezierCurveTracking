#include "bezier_tracking/line_tracking.hpp"

LineTracking::LineTracking(Params params):params_(params){
    lambda_vel = std::sqrt(std::pow(params_.vel,2)/(std::pow(params_.x_end-params_.x_start,2) + std::pow(params_.y_end-params_.y_start,2)));
    if (!params_.forward){
        lambda_vel *= -1;
    }
    theta_diff_max = std::atan(params_.vel*params_.dt/params_.r_min);
    std::cout << "theta_diff_max = " << theta_diff_max << std::endl;
}

void LineTracking::set_horizon(float horizon){
    params_.horizon = horizon;
}


void LineTracking::line_interpolate(float lambda, float &x, float &y){
    x = params_.x_start*(1-lambda) + params_.x_end*lambda;
    y = params_.y_start*(1-lambda) + params_.y_end*lambda;
}

float LineTracking::find_angle(float x1, float y1, float x2, float y2){
    float dot;
    float mag1,mag2;
    float angle;

    dot = (x1*x2) + (y1*y2);
    mag1 = std::sqrt(std::pow(x1,2) + std::pow(y1,2));
    mag2 = std::sqrt(std::pow(x2,2) + std::pow(y2,2));

    angle = std::acos(dot/(mag1*mag2));
    if (x1*y2 - y1*x2 < 0){
        angle *= -1;
    }

    return angle;
}

void LineTracking::rotate(float &x, float &y, float angle){
    float x_temp = x;
    float y_temp = y;

    x = std::cos(angle)*x_temp - std::sin(angle)*y_temp;
    y = std::sin(angle)*x_temp + std::cos(angle)*y_temp;
}

LineTracking::Trajectory LineTracking::pursuitcurve_sim(float x0, float y0, float theta0, float lambda){
    float x_lead,y_lead;
    float x_follow, y_follow;
    float vx_lead,vy_lead;
    float vx_follow,vy_follow;
    float distance,angle,angle_diff;
    float theta_current,theta_next,theta_diff;
    Trajectory trajectory;

    trajectory.pos.clear();
    trajectory.vel.clear();
    trajectory.pos.resize(params_.horizon+1);
    trajectory.vel.resize(params_.horizon+1);
    trajectory.total_cost = 0;

    x_follow = x0;
    y_follow = y0;
    theta_current = theta0;
    for (int i = 0; i <= params_.horizon; i++){
        // Find v_follow
        line_interpolate(lambda,x_lead,y_lead);
        distance = std::sqrt(std::pow(x_lead-x_follow,2) + std::pow(y_lead-y_follow,2));
        vx_lead   = (params_.x_end-params_.x_start)*lambda_vel;
        vy_lead   = (params_.y_end-params_.y_start)*lambda_vel;
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

        // Constrain v_follow w.r.t minimum turning radius
        theta_next = find_angle(1,0,vx_follow,vy_follow);
        theta_diff = theta_current - theta_next;;
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
            theta_current = find_angle(1,0,vx_follow,vy_follow);
        }
        else{
            angle_diff = std::abs(theta_diff);
            theta_current = theta_next;
        }
        
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

LineTracking::Trajectory LineTracking::tracking(float x0, float y0, float theta0){
    Trajectory traj_forward,traj_backward;
    Trajectory traj_best,traj_test;

    traj_best     = pursuitcurve_sim(x0,y0,theta0,lambda);
    traj_forward  = pursuitcurve_sim(x0,y0,theta0,lambda+params_.stepsize);
    traj_backward = pursuitcurve_sim(x0,y0,theta0,lambda-params_.stepsize);

    if (traj_forward.total_cost <= traj_backward.total_cost){
        // Favor forward
        std::cout << "Favor forward\n";
        lambda += params_.stepsize;
        while(true){
            if (traj_best.total_cost < traj_forward.total_cost){
                lambda -= params_.stepsize;
                std::cout << "best lambda = " << lambda << std::endl;
                return traj_best;
            }
            else{
                traj_best    = traj_forward;
                lambda      += params_.stepsize;
                traj_forward = pursuitcurve_sim(x0,y0,theta0,lambda);
            }
        }
    }
    else{
        // Favor backward
        std::cout << "Favor bacward\n";
        lambda -= params_.stepsize;
        while(true){
            if (traj_best.total_cost <= traj_backward.total_cost){
                lambda += params_.stepsize;
                std::cout << "best lambda = " << lambda << std::endl;
                return traj_best;
            }
            else{
                traj_best     = traj_backward;
                lambda       -= params_.stepsize;
                traj_backward = pursuitcurve_sim(x0,y0,theta0,lambda);
            }
        }
    }

}