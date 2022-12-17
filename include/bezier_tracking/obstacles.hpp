#pragma once
#include <cmath>
#include <vector>
#include <string>

class Obstacle{
    public:
        virtual float evaluate(float x, float y) = 0;
        virtual void gradient(float x, float y, float &gx, float &gy) = 0;
        virtual std::vector<float> properties() = 0;
        virtual std::string shape() = 0;
        bool is_collided(float x, float y);

    private:
};

class Circle : public Obstacle {
    public:
        struct Params{
            float xc;
            float yc;
            float r;
        };

        Circle(Params params);
        Circle();

        float evaluate(float x, float y) override;
        void  gradient(float x, float y, float &gx, float &gy) override;
        std::vector<float> properties() override;
        std::string shape() override;

        void update_center(float x, float y);
        void update_radius(float r);

    private:
        Params params_;
};