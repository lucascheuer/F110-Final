#ifndef INPUT_H
#define INPUT_H


#include <ros/ros.h>
#include <Eigen/Geometry>

class Input
{
    public:
        Input();
        void set_v(double v);
        void set_steer_ang(double steer_ang);
        virtual ~Input();
        //getters
        double v();
        double steer_ang();

    private:
        double v_;
        double steer_ang_;
        // mode
};

#endif