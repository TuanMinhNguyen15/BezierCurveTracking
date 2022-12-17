#include "bezier_tracking/bezier_curves.hpp"

std::string BezierCurve::bezier_type(){
    return "undefined";
}

std::vector<std::array<float,2>> BezierCurve::get_control_points(){
    std::vector<std::array<float,2>> control_points;
    return control_points;
}


// Linear Bezier Curves
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


// Map = Combination of Bezier Curves
Map::Map(){}
Map::Map(std::vector<BezierCurve*> curves):curves_(curves){
    num_curves = curves_.size();
}

void Map::update_map(std::vector<BezierCurve*> curves){
    curves_ = curves;
    num_curves = curves_.size();
}

void Map::clear_curves(){
    curves_.clear();
    num_curves = 0;
}

void  Map::get_point(float lambda, float &x, float &y) {
    int index = std::floor(lambda);
    index = std::min(std::max(index,0),num_curves-1);
    lambda -= index;

    curves_[index]->get_point(lambda,x,y);
}

void  Map::get_velocity(float lambda, float lambda_vel, float &vx, float &vy) {
    int index = std::floor(lambda);
    index = std::min(std::max(index,0),num_curves-1);
    lambda -= index;

    curves_[index]->get_velocity(lambda,lambda_vel,vx,vy);
}

float Map::lambda_velocity(float lambda, float velocity) {
    int index = std::floor(lambda);
    index = std::min(std::max(index,0),num_curves-1);
    lambda -= index;

    return curves_[index]->lambda_velocity(lambda,velocity);
}

std::vector<BezierCurve*> Map::get_curves(){
    return curves_;
}

float Map::get_lambda_diff(float lambda){
    int index = std::floor(lambda);
    index = std::min(std::max(index,0),num_curves-1);

    return curves_[index]->lambda_diff;
}

int Map::get_num_curves(){
    return num_curves;
}