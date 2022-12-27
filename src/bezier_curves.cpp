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


// Quadratic Bezier Curves
QuadraticBezier::QuadraticBezier(Params params):params_(params){
    float mag  = std::sqrt(std::pow(params_.x_start-params_.x_inter,2)+std::pow(params_.y_start-params_.y_inter,2)) +
                 std::sqrt(std::pow(params_.x_inter-params_.x_end,2)+std::pow(params_.y_inter-params_.y_end,2));

    lambda_diff = params_.dstep/mag;

    LinearBezier::Params line_params;

    line_params.x_start = params_.x_start;
    line_params.y_start = params_.y_start;
    line_params.x_end   = params_.x_inter;
    line_params.y_end   = params_.y_inter;
    line_before = new LinearBezier(line_params);

    line_params.x_start = params_.x_inter;
    line_params.y_start = params_.y_inter;
    line_params.x_end   = params_.x_end;
    line_params.y_end   = params_.y_end;
    line_after = new LinearBezier(line_params);
}

QuadraticBezier::~QuadraticBezier(){
    delete line_before;
    delete line_after;
}

void  QuadraticBezier::get_point(float lambda, float &x, float &y){
    if (lambda < 0){
        line_before->get_point(lambda,x,y);
    }
    else if (lambda > 1){
        line_after->get_point(lambda,x,y);
    }
    else{
        x = std::pow(1-lambda,2)*params_.x_start + 2*lambda*(1-lambda)*params_.x_inter + std::pow(lambda,2)*params_.x_end;
        y = std::pow(1-lambda,2)*params_.y_start + 2*lambda*(1-lambda)*params_.y_inter + std::pow(lambda,2)*params_.y_end;
    }
}

void  QuadraticBezier::get_velocity(float lambda, float lambda_vel, float &vx, float &vy){
    if (lambda < 0){
        line_before->get_velocity(lambda,lambda_vel,vx,vy);
    }
    else if (lambda > 1){
        line_after->get_velocity(lambda,lambda_vel,vx,vy);
    }
    else{
        vx = (-2*(1-lambda)*params_.x_start + (2-4*lambda)*params_.x_inter + 2*lambda*params_.x_end)*lambda_vel;
        vy =  (-2*(1-lambda)*params_.y_start + (2-4*lambda)*params_.y_inter + 2*lambda*params_.y_end)*lambda_vel;
    }
}

float QuadraticBezier::lambda_velocity(float lambda, float velocity){
    if (lambda < 0){
        return line_before->lambda_velocity(lambda,velocity);
    }
    else if (lambda > 1){
        return line_after->lambda_velocity(lambda,velocity);
    }
    else{
        float lambda_vel;
        lambda_vel = std::sqrt(std::pow(velocity,2)/(std::pow(-2*(1-lambda)*params_.x_start + (2-4*lambda)*params_.x_inter + 2*lambda*params_.x_end,2) + std::pow(-2*(1-lambda)*params_.y_start + (2-4*lambda)*params_.y_inter + 2*lambda*params_.y_end,2)));
        return lambda_vel;
    }
}

std::string QuadraticBezier::bezier_type(){
    return "quadratic";
}

std::vector<std::array<float,2>> QuadraticBezier::get_control_points(){
    std::vector<std::array<float,2>> control_points;

    control_points.resize(3);
    control_points[0][0] = params_.x_start;
    control_points[0][1] = params_.y_start;
    control_points[1][0] = params_.x_inter;
    control_points[1][1] = params_.y_inter;
    control_points[2][0] = params_.x_end;
    control_points[2][1] = params_.y_end;

    return control_points;
}


// Cubic Bezier Curves
CubicBezier::CubicBezier(Params params):params_(params){
    float mag  = std::sqrt(std::pow(params_.x_start-params_.x_inter1,2)+std::pow(params_.y_start-params_.y_inter1,2)) +
                 std::sqrt(std::pow(params_.x_inter1-params_.x_inter2,2)+std::pow(params_.y_inter1-params_.y_inter2,2)) + 
                 std::sqrt(std::pow(params_.x_inter2-params_.x_end,2)+std::pow(params_.y_inter2-params_.y_end,2));

    lambda_diff = params_.dstep/mag;

    LinearBezier::Params line_params;

    line_params.x_start = params_.x_start;
    line_params.y_start = params_.y_start;
    line_params.x_end   = params_.x_inter1;
    line_params.y_end   = params_.y_inter1;
    line_before = new LinearBezier(line_params);

    line_params.x_start = params_.x_inter2;
    line_params.y_start = params_.y_inter2;
    line_params.x_end   = params_.x_end;
    line_params.y_end   = params_.y_end;
    line_after = new LinearBezier(line_params);
}

CubicBezier::~CubicBezier(){
    delete line_before;
    delete line_after;
}

void  CubicBezier::get_point(float lambda, float &x, float &y){
    if (lambda < 0){
        line_before->get_point(lambda,x,y);
    }
    else if (lambda > 1){
        line_after->get_point(lambda,x,y);
    }
    else{
        x = std::pow(1-lambda,3)*params_.x_start + 3*std::pow(1-lambda,2)*lambda*params_.x_inter1 + 3*(1-lambda)*std::pow(lambda,2)*params_.x_inter2 + std::pow(lambda,3)*params_.x_end;
        y = std::pow(1-lambda,3)*params_.y_start + 3*std::pow(1-lambda,2)*lambda*params_.y_inter1 + 3*(1-lambda)*std::pow(lambda,2)*params_.y_inter2 + std::pow(lambda,3)*params_.y_end;
    }
}

void  CubicBezier::get_velocity(float lambda, float lambda_vel, float &vx, float &vy){
    if (lambda < 0){
        line_before->get_velocity(lambda,lambda_vel,vx,vy);
    }
    else if (lambda > 1){
        line_after->get_velocity(lambda,lambda_vel,vx,vy);
    }
    else{
        vx = (-3*std::pow(1-lambda,2)*params_.x_start + (-6*(1-lambda)*lambda + 3*std::pow(1-lambda,2))*params_.x_inter1 + (-3*std::pow(lambda,2) + 6*(1-lambda)*lambda)*params_.x_inter2 + 3*std::pow(lambda,2)*params_.x_end)*lambda_vel;
        vy = (-3*std::pow(1-lambda,2)*params_.y_start + (-6*(1-lambda)*lambda + 3*std::pow(1-lambda,2))*params_.y_inter1 + (-3*std::pow(lambda,2) + 6*(1-lambda)*lambda)*params_.y_inter2 + 3*std::pow(lambda,2)*params_.y_end)*lambda_vel;
    }
}

float CubicBezier::lambda_velocity(float lambda, float velocity){
    if (lambda < 0){
        return line_before->lambda_velocity(lambda,velocity);
    }
    else if (lambda > 1){
        return line_after->lambda_velocity(lambda,velocity);
    }
    else{
        float lambda_vel;
        lambda_vel = std::sqrt(std::pow(velocity,2)/(std::pow(-3*std::pow(1-lambda,2)*params_.x_start + (-6*(1-lambda)*lambda + 3*std::pow(1-lambda,2))*params_.x_inter1 + (-3*std::pow(lambda,2) + 6*(1-lambda)*lambda)*params_.x_inter2 + 3*std::pow(lambda,2)*params_.x_end,2) + 
                                                     std::pow(-3*std::pow(1-lambda,2)*params_.y_start + (-6*(1-lambda)*lambda + 3*std::pow(1-lambda,2))*params_.y_inter1 + (-3*std::pow(lambda,2) + 6*(1-lambda)*lambda)*params_.y_inter2 + 3*std::pow(lambda,2)*params_.y_end,2)));
        return lambda_vel;
    }
}

std::string CubicBezier::bezier_type(){
    return "cubic";
}

std::vector<std::array<float,2>> CubicBezier::get_control_points(){
    std::vector<std::array<float,2>> control_points;

    control_points.resize(4);
    control_points[0][0] = params_.x_start;
    control_points[0][1] = params_.y_start;
    control_points[1][0] = params_.x_inter1;
    control_points[1][1] = params_.y_inter1;
    control_points[2][0] = params_.x_inter2;
    control_points[2][1] = params_.y_inter2;
    control_points[3][0] = params_.x_end;
    control_points[3][1] = params_.y_end;

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