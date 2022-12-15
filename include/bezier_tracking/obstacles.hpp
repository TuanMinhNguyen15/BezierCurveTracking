#pragma once
#include <cmath>

class Obstacle{
    public:
        virtual float evaluate(float x, float y) = 0;
        virtual void gradient(float x, float y, float &gx, float &gy) = 0;
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

        float evaluate(float x, float y) override;
        void  gradient(float x, float y, float &gx, float &gy) override;

    private:
        Params params_;
};