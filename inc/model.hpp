#ifndef MODEL_H
#define MODEL_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include "state.hpp"
#include "input.hpp"


class Model
{
    public:
        Model();
        Eigen::MatrixXd a();
        Eigen::MatrixXd b();
        Eigen::MatrixXd c();
        void linearize(State &S, Input &I, double dt);
        virtual ~Model();
        
    private:
        
        double time_step_;
        Eigen::MatrixXd a_;
        Eigen::MatrixXd b_;
        Eigen::MatrixXd c_;
        // mode
};

#endif