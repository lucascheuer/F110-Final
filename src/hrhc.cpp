#include "hrhc.hpp"



HRHC::~HRHC()
{
    ROS_INFO("Killing HRHC");
}

HRHC:: HRHC(ros::NodeHandle &nh): nh_(nh), occ_grid_(nh, 10,0.1), trajp_(nh)
{
    
    std::string pose_topic, scan_topic;
    pose_topic = "/odom";
    scan_topic = "/scan";
    pf_sub_ = nh_.subscribe(pose_topic, 10, &HRHC::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &HRHC::scan_callback, this);
    current_pose.position.x = 0;
    current_pose.position.y = 0;
    current_pose.position.z = 0;
    current_pose.orientation.x = 0;
    current_pose.orientation.y = 0;
    current_pose.orientation.z = 0;
    current_pose.orientation.w = 1;
    // trajp_ = TrajectoryPlanner(nh);
    trajp_.getTrajectories();
    trajp_.trajectory2world(current_pose);
    trajp_.trajectory2miniworld(current_pose);
    trajp_.getCmaes();
    mpc_.DoMPC();
    ROS_INFO("Created HRHC");
}

void HRHC::pf_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{   
    current_pose = odom_msg->pose.pose;
    firstPoseEstimate = true;
    

    trajp_.Update(current_pose, occ_grid_);
    trajp_.Visualize();
}

void HRHC::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (firstPoseEstimate)
    {   
        occ_grid_.FillOccGrid(current_pose, scan_msg, 0.1f);
        occ_grid_.Visualize();
        
    }
}

