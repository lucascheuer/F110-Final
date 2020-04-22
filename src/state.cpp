#include "state.hpp"

State::State(): x_(0), y_(0), ori_(0), size_(3)
{
    // ROS_INFO("state created");
}

State::State(double x, double y, double ori): x_(x), y_(y), ori_(ori), size_(3)
{
    // ROS_INFO("state created");
}

State::~State()
{
    // ROS_INFO("killing the state");
}

Eigen::VectorXd State::ToVector()
{
    Eigen::VectorXd state_vector;
    state_vector.resize(size_);
    state_vector << x_, y_, ori_;
    return state_vector;
}
// Mutators
void State::SetX(double x)
{
    x_ = x;
}

void State::SetY(double y)
{
    y_ = y;
}

void State::SetOri(double ori)
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

int State::size()
{
    return size_;
}