#ifndef INPUT_H
#define INPUT_H


#include <ros/ros.h>
#include <Eigen/Geometry>

// Specifies the format for the Input object of velocity and steering angle

class Input
{
    public:
        Input();
        Input(double v, double steer_ang);
        virtual ~Input();
        Eigen::VectorXd ToVector();

        void SetV(double v);
        void SetSteerAng(double steer_ang);
        //getters
        double v();
        double steer_ang();

    private:
        double v_;
        double steer_ang_;
        int size_;
        // mode
};

#endif