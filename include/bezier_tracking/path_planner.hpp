#include <vector>
#include <iostream>
#include <array>
#include <cmath>
#include "bezier_tracking/map.hpp"
#include "bezier_tracking/obstacles.hpp"

class PathPlanner{
    public:
        struct Params{
            Map *map;
            float vel;
            float r_min;
            float dt = 0.01;
            int   horizon = 10;
            float distance_cost = 1;
            float heading_cost  = 1;
            float smooth_cost   = 1;

            std::vector<Obstacle*> obstacles;
            float reactivity = 1;
        };

        struct Trajectory{
            std::vector<std::array<float,2>> pos,vel;
            float total_cost;

        };

        PathPlanner(Params params);

        void update_map(Map *map);
        void update_lambda(float lambda);
        float get_dt();
        float find_angle(float x1, float y1, float x2, float y2);
        void rotate(float &x, float &y, float angle);
        Trajectory pursuitcurve_sim(float x0, float y0, float theta0, float lambda);
        Trajectory tracking(float x0, float y0, float theta0);
        void priming(float x0, float y0, float theta0);

        void obstacle_avoidance(float x0, float y0, float theta0, float &vx, float &vy);

    private:
        Params params_;
        float lambda = 0;
        float theta_diff_max;
};