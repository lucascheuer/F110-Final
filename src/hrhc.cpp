#include "hrhc.hpp"


HRHC::~HRHC()
{
    ROS_INFO("Killing HRHC");
}

HRHC:: HRHC(ros::NodeHandle &nh): nh_(nh)
{
    ROS_INFO("Created HRHC");
}