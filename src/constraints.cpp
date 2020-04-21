#include "constraints.hpp"

Constraints::Constraints()
{
    x_max_.resize(3,1);
    x_max_ << 1000.0f,1000.0f,1000.0f; //x,y,ori
    x_min_.resize(3,1);
    x_min_ << -1000.0f,-1000.0f,-1000.0f; //x,y,ori
    u_max_.resize(2,1);
    u_max_ << 4.5f, 0.43f; //Speed, steering
    u_min_.resize(2,1);
    u_min_ << 0.0f, -0.43f; //Speed, steering
    d = 1.0f;
    ROS_INFO("constraints created");
}

Constraints::~Constraints()
{
    ROS_INFO("killing the constraints");
}


Eigen::MatrixXf Constraints::x_max()
{
    return x_max_;
}

Eigen::MatrixXf Constraints::x_min()
{
    return x_min_;
}

Eigen::MatrixXf Constraints::u_max()
{
    return u_max_;
}

Eigen::MatrixXf Constraints::u_min()
{
    return u_min_;
}


void Constraints::set_x_max(Eigen::MatrixXf xmax)
{
    x_max_.resize(3,1);
    x_max_ = xmax;
}

void Constraints::set_u_max(Eigen::MatrixXf umax)
{
    u_max_.resize(2,1);
    u_max_ = umax;
}

void Constraints::set_x_min(Eigen::MatrixXf xmin)
{
    x_min_.resize(3,1);
    x_min_ = xmin;
}

void Constraints::set_u_min(Eigen::MatrixXf umin)
{
    u_min_.resize(2,1);
    u_min_ = umin;
}

void Constraints::SetXLims(float x,float y)
{
    x_max_(0,0) = x+d;
    x_max_(1,0) = y+d;
    x_min_(0,0) = x-d;
    x_min_(1,0) = y-d;
}
