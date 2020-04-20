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
        Eigen::MatrixXf getA();
        Eigen::MatrixXf getB();
        Eigen::MatrixXf getC();
        void linearize(State &S, Input &I, double dt);
        virtual ~Model();
    private:
        Eigen::MatrixXf a_;
        Eigen::MatrixXf b_;
        Eigen::MatrixXf c_;
        double time_step_;
        // mode
};

#endif