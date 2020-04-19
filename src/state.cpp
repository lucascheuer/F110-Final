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
    x_ = y;
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