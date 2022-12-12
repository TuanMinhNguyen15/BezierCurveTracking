#include "bezier_tracking/map.hpp"

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