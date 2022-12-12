#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <array>

class BezierCurve{
    public:
        virtual void  get_point(float lambda, float &x, float &y) = 0;
        virtual void  get_velocity(float lambda, float lambda_vel, float &vx, float &vy) = 0;
        virtual float lambda_velocity(float lambda, float velocity) = 0;
        virtual std::string bezier_type();
        virtual std::vector<std::array<float,2>> get_control_points();

        float lambda_diff = 0;
};