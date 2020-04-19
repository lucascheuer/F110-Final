#include "hrhc.hpp"
#include <cmath>


HRHC::~HRHC()
{
    ROS_INFO("Killing HRHC");
}

HRHC:: HRHC(ros::NodeHandle &nh): nh_(nh),occgrid(10,0.1)
{
    ROS_INFO("Created HRHC");
    std::string pose_topic, scan_topic;
    pose_topic = "/gt_pose";
    scan_topic = "/scan";
    pf_sub_ = nh_.subscribe(pose_topic, 10, &HRHC::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &HRHC::scan_callback, this);
    vis_pub_mult = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1 );
    trajp.getTrajectories();
}

void HRHC::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{   
    current_pose = pose_msg->pose;
    firstPoseEstimate = true;
    trajp.trajectory2world(current_pose);
    visualization_msgs::MarkerArray local_traj_markers = trajp.gen_markers(trajp.trajectories_world);
    vis_pub_mult.publish( local_traj_markers );
}

void HRHC::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (firstPoseEstimate)
    {   
        occgrid.fill_occgrid(current_pose, scan_msg, 0.1f);
    }
}

