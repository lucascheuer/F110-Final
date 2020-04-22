#include "input.hpp"

Input::Input()
{
    // ROS_INFO("input created");
}

Input::Input(double v, double steer_ang): v_(v), steer_ang_(steer_ang), size_(2)
{
    // ROS_INFO("state created");
}

Input::~Input()
{
    // ROS_INFO("killing the input");
}

Eigen::VectorXd Input::ToVector()
{
    Eigen::VectorXd input_vector;
    input_vector.resize(size_);
    input_vector << v_, steer_ang_;
    return input_vector;
}

// Mutators
void Input::SetV(double v)
{
    v_ = v;
}

void Input::SetSteerAng(double steer_ang)
{
    steer_ang_ = steer_ang;
}

// Accessors
double Input::v()
{
    return v_;
}

double Input::steer_ang()
{
    return steer_ang_;
}