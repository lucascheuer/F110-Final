#include "constraints.hpp"
Constraints::Constraints()
{
    
    x_max_.resize(3,1);
    x_max_ << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY; //x,y,ori
    x_min_.resize(3,1);
    x_min_ << -OsqpEigen::INFTY,-OsqpEigen::INFTY,-OsqpEigen::INFTY; //x,y,ori
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


Eigen::VectorXd Constraints::x_max()
{
    return x_max_;
}

Eigen::VectorXd Constraints::x_min()
{
    return x_min_;
}

Eigen::VectorXd Constraints::u_max()
{
    return u_max_;
}

Eigen::VectorXd Constraints::u_min()
{
    return u_min_;
}

Eigen::VectorXd Constraints::MovedXMax()
{
    return x_max_ += state_.ToVector();
}

Eigen::VectorXd Constraints::MovedXMin()
{
    return x_min_ += state_.ToVector();
}



void Constraints::set_x_max(Eigen::VectorXd xmax)
{
    x_max_.resize(xmax.size());
    x_max_ = xmax;
}

void Constraints::set_u_max(Eigen::VectorXd umax)
{
    u_max_.resize(umax.size());
    u_max_ = umax;
}

void Constraints::set_x_min(Eigen::VectorXd xmin)
{
    x_min_.resize(xmin.size());
    x_min_ = xmin;
}

void Constraints::set_u_min(Eigen::VectorXd umin)
{
    u_min_.resize(umin.size());
    u_min_ = umin;
}

void Constraints::set_state(State state)
{
    state_ = state;
}

void Constraints::SetXLims(State state)
{
    x_max_(0,0) = state.x()+d;
    x_max_(1,0) = state.y()+d;
    x_min_(0,0) = state.x()-d;
    x_min_(1,0) = state.y()-d;
}
