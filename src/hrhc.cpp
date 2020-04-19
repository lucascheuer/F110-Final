#include "hrhc.hpp"
#include <cmath>


HRHC::~HRHC()
{
    ROS_INFO("Killing HRHC");
}

HRHC:: HRHC(ros::NodeHandle &nh): nh_(nh)
{
    ROS_INFO("Created HRHC");
    std::string pose_topic, scan_topic;
    pf_sub_ = nh_.subscribe(pose_topic, 10, &HRHC::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &HRHC::scan_callback, this);
    
}

void HRHC::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    current_pose = pose_msg->pose;
    firstPoseEstimate = true;
}

void HRHC::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (firstPoseEstimate)
    {   
        OccGrid occgrid = OccGrid(10,0.1);
        occgrid.fill_occgrid(current_pose, scan_msg, 0.1f);
       
    }
}

