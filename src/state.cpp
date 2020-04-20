#include "state.hpp"

State::State()
{
    ROS_INFO("state created");
}

State::~State()
{
    ROS_INFO("killing the state");
}


// Mutators
void State::set_x(double x)
{
    x_ = x;
}

void State::set_y(double y)
{
    y_ = y;
}

void State::set_ori(double ori)
{
    ori_ = ori;
}

// Accessors
double State::x()
{
    return x_;
}

double State::y()
{
    return y_;
}

double State::ori()
{
    return ori_;
}