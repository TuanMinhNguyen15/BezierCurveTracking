#pragma once
#include "bezier_tracking/bezier_curve.hpp"

// Linear Bezier Curve
class LinearBezier : public BezierCurve{
    public:
        struct Params{
            float x_start,y_start;
            float x_end,y_end;
            float dstep = 0.1;
        };

        LinearBezier(Params params);

        void  get_point(float lambda, float &x, float &y) override;
        void  get_velocity(float lambda, float lambda_vel, float &vx, float &vy) override;
        float lambda_velocity(float lambda, float velocity) override;
        std::string bezier_type() override;
        std::vector<std::array<float,2>> get_control_points() override;

    private:
        Params params_;
};