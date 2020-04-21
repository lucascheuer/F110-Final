#include "cost.hpp"

Cost::Cost()
{
}
Cost::Cost(Eigen::MatrixXd q, Eigen::MatrixXd r): q_(q), r_(r)
{
    ROS_INFO("cost created");
}

Cost::~Cost()
{
    ROS_INFO("killing the cost");
}

void Cost::SetQ(Eigen::MatrixXd q)
{
    q_ = q;
}

void Cost::SetR(Eigen::MatrixXd r)
{
    r_ = r;
}

Eigen::MatrixXd Cost::q()
{
    return q_;
}
Eigen::MatrixXd Cost::r()
{
    return r_;
}