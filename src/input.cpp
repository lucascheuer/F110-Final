#include "input.hpp"

Input::Input()
{
    ROS_INFO("input created");
}

Input::~Input()
{
    ROS_INFO("killing the input");
}

// Mutators
void Input::set_v(double v)
{
    v_ = v;
}

void Input::set_steer_ang(double steer_ang)
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