#include "hrhc.hpp"



HRHC::~HRHC()
{
    ROS_INFO("Killing HRHC");
}

HRHC:: HRHC(ros::NodeHandle &nh): nh_(nh), occgrid(nh, 10,0.1)
{
    
    std::string pose_topic, scan_topic;
    pose_topic = "/odom";
    scan_topic = "/scan";
    pf_sub_ = nh_.subscribe(pose_topic, 10, &HRHC::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &HRHC::scan_callback, this);
    vis_pub_mult = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1 );
    current_pose.position.x = 0;
    current_pose.position.y = 0;
    current_pose.position.z = 0;
    current_pose.orientation.x = 0;
    current_pose.orientation.y = 0;
    current_pose.orientation.z = 0;
    current_pose.orientation.w = 1;
    trajp = TrajectoryPlanner();
    trajp.getTrajectories();
    trajp.trajectory2world(current_pose);
    trajp.trajectory2miniworld(current_pose);
    trajp.getCmaes();
    ROS_INFO("Created HRHC");
}

void HRHC::pf_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{   
    current_pose = odom_msg->pose.pose;
    firstPoseEstimate = true;
    trajp.trajectory2miniworld(current_pose);
    trajp.trajectory2world(current_pose);
    

}

void HRHC::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (firstPoseEstimate)
    {   

        occgrid.FillOccGrid(current_pose, scan_msg, 0.1f);
        occgrid.Visualize();
        int best_traj_index = trajp.best_traj(occgrid,current_pose);
        std::vector<pair<float,float>> best_traj;
        for (int i = 10*best_traj_index; i<10*best_traj_index+10;i++)
        {
            best_traj.push_back(trajp.trajectories_world[i]);
        }

        visualization_msgs::MarkerArray local_traj_markers = trajp.gen_markers(best_traj);
        vis_pub_mult.publish( local_traj_markers );
    }
}

