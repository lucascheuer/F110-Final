#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include "occgrid.hpp"
#include "visualizer.hpp"
#include <nav_msgs/Odometry.h>
#include "trajectory_planner.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "OsqpEigen/OsqpEigen.h"

class HRHC
{
    public:
        HRHC(ros::NodeHandle &nh);
        virtual ~HRHC();
    private:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        ros::Subscriber pf_sub_;

        bool firstPoseEstimate;
        geometry_msgs::Pose current_pose;
        std::pair<float, float> occ_offset;
        OccGrid occgrid;
        TrajectoryPlanner trajp;
        

        void pf_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};