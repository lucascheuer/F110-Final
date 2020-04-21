#include "constraints.hpp"

Constraints::Constraints()
{
    XMax_.resize(3,1);
    XMax_ << 1000.0f,1000.0f,1000.0f; //x,y,ori
    XMin_.resize(3,1);
    XMin_ << -1000.0f,-1000.0f,-1000.0f; //x,y,ori
    UMax_.resize(2,1);
    UMax_ << 4.5f, 0.43f; //Speed, steering
    UMin_.resize(2,1);
    UMin_ << 0.0f, -0.43f; //Speed, steering
    d = 1.0f;
    ROS_INFO("constraints created");
}

Constraints::~Constraints()
{
    ROS_INFO("killing the constraints");
}


Eigen::MatrixXf Constraints::getXMax()
{
    return XMax_;
}

Eigen::MatrixXf Constraints::getXMin()
{
    return XMin_;
}

Eigen::MatrixXf Constraints::getUMax()
{
    return UMax_;
}

Eigen::MatrixXf Constraints::getUMin()
{
    return UMin_;
}


void Constraints::setXMax(Eigen::MatrixXf xmax)
{
    XMax_.resize(3,1);
    XMax_ = xmax;
}

void Constraints::setXMin(Eigen::MatrixXf umax)
{
    UMax_.resize(2,1);
    UMax_ = umax;
}

void Constraints::setUMax(Eigen::MatrixXf xmin)
{
    XMin_.resize(3,1);
    XMin_ = xmin;
}

void Constraints::setUMin(Eigen::MatrixXf umin)
{
    UMin_.resize(2,1);
    UMin_ = umin;
}

void Constraints::setXlims(float x,float y)
{
    XMax_(0,0) = x+d;
    XMax_(1,0) = y+d;
    XMin_(0,0) = x-d;
    XMin_(1,0) = y-d;
}
