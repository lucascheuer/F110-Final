#ifndef COST_H
#define COST_H
#include <ros/ros.h>
#include <Eigen/Geometry>

class Cost
{
    public:
        Cost();
        Cost(Eigen::MatrixXd q, Eigen::MatrixXd r);
        virtual ~Cost();

        void SetQ(Eigen::MatrixXd q);
        void SetR(Eigen::MatrixXd r);
        Eigen::MatrixXd q();
        Eigen::MatrixXd r();
    private:
        Eigen::MatrixXd q_;
        Eigen::MatrixXd r_;
        // mode
};
#endif