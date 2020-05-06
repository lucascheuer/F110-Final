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

// Main class which loads objects of other classes

class HRHC
{
public:
    HRHC(ros::NodeHandle &nh);
    virtual ~HRHC();

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher drive_pub_;
    bool first_pose_estimate_ = false;
    bool first_scan_estimate_ = false;

    geometry_msgs::Pose current_pose_;
    std::pair<float, float> occ_offset_;
    OccGrid occ_grid_;
    TrajectoryPlanner trajp_;
    MPC mpc_;
    State mpc_des_state_;
    std::vector<Input> current_inputs_;
    std::atomic<unsigned int> inputs_idx_;

    // Infinite loop for publishing /drive messages
    void DriveLoop();
    // Updates MPC object and saves the currently solved trajectory
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    // Updates Occupancy grid
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    // Used to provide the next input for /drive
    Input GetNextInput();
};