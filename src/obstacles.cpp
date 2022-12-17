#include "bezier_tracking/obstacles.hpp"

bool Obstacle::is_collided(float x, float y){
    if (evaluate(x,y) <= 1){
        return true;
    }
    else{
        return false;
    }
}

Circle::Circle(Params params):params_(params){}

Circle::Circle(){
    params_.xc = 0;
    params_.yc = 0;
    params_.r = 0;
}

float Circle::evaluate(float x, float y){
    return (std::pow(x-params_.xc,2) + std::pow(y-params_.yc,2))/std::pow(params_.r,2);
}

void  Circle::gradient(float x, float y, float &gx, float &gy){
    float mag; 

    gx = x - params_.xc;
    gy = y - params_.yc;

    mag = std::sqrt(std::pow(gx,2) + std::pow(gy,2));

    gx = gx/mag;
    gy = gy/mag;
}

std::vector<float> Circle::properties(){
    std::vector<float> prop = {params_.xc,params_.yc,params_.r};
    return prop;
}

std::string Circle::shape(){
    return "circle";
}

void Circle::update_center(float x, float y){
    params_.xc = x;
    params_.yc = y;
}

void Circle::update_radius(float r){
    params_.r = r;
}