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


// Linear Bezier Curves
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


// Map = Combination of Bezier Curves
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
        std::vector<BezierCurve*> curves_;
};