#pragma once
#include <vector>
#include <map>
#include "bezier_tracking/bezier_curve.hpp"

class Map : public BezierCurve{
    public:
        Map();
        Map(std::vector<BezierCurve*> curves);

        void update_map(std::vector<BezierCurve*> curves);
        void clear_curves();

        void  get_point(float lambda, float &x, float &y) override;
        void  get_velocity(float lambda, float lambda_vel, float &vx, float &vy) override;
        float lambda_velocity(float lambda, float velocity) override;
        
        float get_lambda_diff(float lambda);
        int   get_num_curves();
        std::vector<BezierCurve*> get_curves();
        
    private:
        int num_curves = 0;
        std::map<int,BezierCurve*> curves_map;
        std::vector<BezierCurve*> curves_;
};