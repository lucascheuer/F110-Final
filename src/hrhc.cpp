#include "hrhc.hpp"



HRHC::~HRHC()
{
    ROS_INFO("Killing HRHC");
}

HRHC:: HRHC(ros::NodeHandle &nh): nh_(nh), occ_grid_(nh, 10,0.1), trajp_(nh), mpc_(5)
{
    
    std::string pose_topic, scan_topic;
    pose_topic = "/odom";
    scan_topic = "/scan";
    pf_sub_ = nh_.subscribe(pose_topic, 10, &HRHC::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &HRHC::scan_callback, this);
    current_pose_.position.x = 0;
    current_pose_.position.y = 0;
    current_pose_.position.z = 0;
    current_pose_.orientation.x = 0;
    current_pose_.orientation.y = 0;
    current_pose_.orientation.z = 0;
    current_pose_.orientation.w = 1;
    // trajp_ = TrajectoryPlanner(nh);
    trajp_.getTrajectories();
    trajp_.trajectory2world(current_pose_);
    trajp_.trajectory2miniworld(current_pose_);
    trajp_.getCmaes();
    Eigen::DiagonalMatrix<double, 3> q;
    Eigen::DiagonalMatrix<double, 2> r;
    q.diagonal() << 1.0, 1.0, 0.0;
    r.diagonal() << 0.00000001, 0.00000001;
    Cost cost(q, r);
    Input input(2, 0.5);
    Model model;
    Constraints constraints;
    mpc_.Init(model, cost, constraints);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
    ROS_INFO("Created HRHC");
}

void HRHC::pf_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{   
    current_pose_ = odom_msg->pose.pose;
    if (!firstPoseEstimate)
        firstPoseEstimate = true;
    float current_angle = atan2(2 * current_pose_.orientation.w * current_pose_.orientation.z, 1 - 2 * current_pose_.orientation.z * current_pose_.orientation.z);
    State current_state(current_pose_.position.x, current_pose_.position.y, current_angle);
    State desired_state(0, 0, 0);
    Input desired_input(2.0, 0);
    
    trajp_.Update(current_pose_, occ_grid_);
    trajp_.Visualize();
    // cout << trajp_.best_cmaes_point_.ToVector() << endl;
    mpc_.Update( current_state, trajp_.best_cmaes_point_, desired_input);
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = mpc_.solved_input().v();
    drive_msg.drive.steering_angle = mpc_.solved_input().steer_ang();
    drive_pub_.publish(drive_msg);
    cout << mpc_.solved_input().v() << "\t" << drive_msg.drive.steering_angle << endl;
}

void HRHC::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (firstPoseEstimate)
    {   
        occ_grid_.FillOccGrid(current_pose_, scan_msg, 0.1f);
        occ_grid_.Visualize();
        
    }
}

