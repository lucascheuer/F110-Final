#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include "occgrid.hpp"
#include <geometry_msgs/Pose.h>
#include "trajectory_planner.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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
        ros::Publisher vis_pub_mult;

        void pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};