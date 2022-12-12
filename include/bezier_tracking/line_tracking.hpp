#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <cmath>

class LineTracking {
    public:
        struct Params{
            // Start and end points of line
            float x_start,x_end;
            float y_start,y_end;
            // Target velocity 
            float vel;
            // Minimum turning radius
            float r_min;
            // Direction
            bool forward  = true;  // true:start->end  false:end->start
            // Discretized time
            float dt = 0.01;
            // Number of timesteps
            float horizon = 10;
            // Cost weights
            float distance_cost = 1;
            float heading_cost  = 1;
            float smooth_cost   = 1;

            // Gradient descent param
            float stepsize = 0.1;
        };

        struct Trajectory{
            std::vector<std::array<float,2>> pos,vel;
            float total_cost;

        };

        LineTracking(Params params);

        void set_horizon(float horizon);

        void line_interpolate(float lambda, float &x, float &y);

        float find_angle(float x1, float y1, float x2, float y2);

        void rotate(float &x, float &y, float angle);

        Trajectory pursuitcurve_sim(float x0, float y0, float theta0, float lambda);

        Trajectory tracking(float x0, float y0, float theta0);

    private:
        Params params_;
        // Line parametrization
        float lambda = 0; 
        float lambda_vel;
        float theta_diff_max;
};