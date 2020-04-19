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
        occgrid.fill_occgrid(current_pose, scan_msg)
        // float current_angle = atan2(2 * current_pose.orientation.w * current_pose.orientation.z, 1 - 2 * current_pose.orientation.z * current_pose.orientation.z);
        // occ_offset.first = current_pose.position.x + 0.275 * cos(current_angle);
        // occ_offset.second = current_pose.position.y + 0.275 * sin(current_angle);
        // int num_scans = (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment + 1;
        // for (int ii = 0; ii < num_scans; ++ii)
        // {
            
        //     float angle = scan_msg->angle_min + ii * scan_msg->angle_increment + current_angle;
        //     std::pair<float, float> cartesian = polar_to_cartesian(scan_msg->ranges[ii], angle);

        //     for (float x_off = -obstacle_dilation; x_off <= obstacle_dilation; x_off += grid_discrete)
        //     {
        //         for (float y_off = -obstacle_dilation; y_off <= obstacle_dilation; y_off += grid_discrete)
        //         {
        //             //cartesian = polar_to_cartesian(scan_msg->ranges[ii] + jj, angle);
        //             cartesian_to_occupancy(cartesian.first + x_off, cartesian.second + y_off);
        //         }
        //     }
    }
}

