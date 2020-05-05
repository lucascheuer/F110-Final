#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include "occgrid.hpp"
#include "visualizer.hpp"
#include "transforms.hpp"
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "trajectory_planner.hpp"
#include "mpc.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "OsqpEigen/OsqpEigen.h"
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
class HRHC
{
    public:
        HRHC(ros::NodeHandle &nh);
        virtual ~HRHC();
        
    private:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        ros::Subscriber nav_sub_;
        ros::Subscriber pf_sub_;
        float q0_;
        float q1_;
        float q2_;
        float r0_;
        float r1_;
        ros::Publisher drive_pub_;
        bool firstPoseEstimate = false;
        bool firstScanEstimate = false;
        
        geometry_msgs::Pose current_pose_;
        std::pair<float, float> occ_offset_;
        OccGrid occ_grid_;
        TrajectoryPlanner trajp_;
        MPC mpc_;
        State mpc_des_state_;
        std::vector<Input> current_inputs_;
        std::atomic<int> inputs_idx_;

        void drive_loop();
        void pf_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
        void nav_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &nav_goal);
        Input get_next_input();
};