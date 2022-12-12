#include "bezier_tracking/linear_bezier.hpp"

LinearBezier::LinearBezier(Params params):params_(params){
    lambda_diff = std::sqrt(std::pow(params_.dstep,2)/(std::pow(params_.x_end-params_.x_start,2)+std::pow(params_.y_end-params_.y_start,2)));
}

void LinearBezier::get_point(float lambda, float &x, float &y) {
    x = params_.x_start*(1-lambda) + params_.x_end*lambda;
    y = params_.y_start*(1-lambda) + params_.y_end*lambda;
}

void LinearBezier::get_velocity(float lambda, float lambda_vel, float &vx, float &vy) {
    vx = (params_.x_end-params_.x_start)*lambda_vel;
    vy = (params_.y_end-params_.y_start)*lambda_vel;
}

float LinearBezier::lambda_velocity(float lambda, float velocity) {
    float lambda_vel;
    lambda_vel = std::sqrt(std::pow(velocity,2)/(std::pow(params_.x_end-params_.x_start,2) + std::pow(params_.y_end-params_.y_start,2)));
    return lambda_vel;
}

std::string LinearBezier::bezier_type(){
    return "linear";
} 

std::vector<std::array<float,2>> LinearBezier::get_control_points(){
    std::vector<std::array<float,2>> control_points;

    control_points.resize(2);
    control_points[0][0] = params_.x_start;
    control_points[0][1] = params_.y_start;
    control_points[1][0] = params_.x_end;
    control_points[1][1] = params_.y_end;

    return control_points;
}