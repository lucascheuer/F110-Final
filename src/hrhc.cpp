#include "hrhc.hpp"



HRHC::~HRHC()
{
    ROS_INFO("Killing HRHC");
}
int hori = 50;
HRHC:: HRHC(ros::NodeHandle &nh):  occ_grid_(nh), trajp_(nh), mpc_(nh)
{
    std::string pose_topic, scan_topic, drive_topic;
    int og_size;
    float discrete;
    
    nh_.getParam("/q0", q0_);
    nh_.getParam("/q1", q1_);
    nh_.getParam("/q2", q2_);
    nh_.getParam("/r0", r0_);
    nh_.getParam("/r1", r1_);
    nh_.getParam("/pose_topic", pose_topic);
    nh_.getParam("/scan_topic", scan_topic);
    nh_.getParam("/drive_topic", drive_topic);
    nh_.getParam("/horizon", hori);
    nh_.getParam("/occgrid_size", og_size);
    nh_.getParam("/occgrid_disc", discrete);
    

    ros::NodeHandle nh_(nh);
    // OccGrid occ_grid_(nh, og_size,discrete);
    pf_sub_ = nh_.subscribe(pose_topic, 1, &HRHC::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 1, &HRHC::scan_callback, this);
    nav_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &HRHC::nav_goal_callback, this);
    current_pose_.position.x = 0;
    current_pose_.position.y = 0;
    current_pose_.position.z = 0;
    current_pose_.orientation.x = 0;
    current_pose_.orientation.y = 0;
    current_pose_.orientation.z = 0;
    current_pose_.orientation.w = 1;
    // trajp_ = TrajectoryPlanner(nh,10);
    trajp_.getTrajectories();
    trajp_.trajectory2world(current_pose_);
    trajp_.trajectory2miniworld(current_pose_);
    trajp_.getCmaes();
    Eigen::DiagonalMatrix<double, 3> q;
    Eigen::DiagonalMatrix<double, 2> r;
    q.diagonal() << q0_, q1_, q2_;
    r.diagonal() << r0_, r1_;
    Cost cost(q, r);
    Model model;
    Constraints constraints(nh);
    mpc_.Init(model, cost, constraints);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
    std::thread t(&HRHC::drive_loop, this);
    t.detach();
    ROS_INFO("Created HRHC");
}

void HRHC::pf_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{   
    current_pose_ = odom_msg->pose.pose;
    if (!firstPoseEstimate) {
        firstPoseEstimate = true;
    }
    float current_angle = atan2(2 * current_pose_.orientation.w * current_pose_.orientation.z, 1 - 2 * current_pose_.orientation.z * current_pose_.orientation.z);
    State current_state(current_pose_.position.x, current_pose_.position.y, current_angle);
    // State desired_state(trajp_.best_cmaes_point_., 0, 0);
    
    trajp_.Update(current_pose_, occ_grid_);
    trajp_.Visualize();
    // cout << trajp_.best_cmaes_point_.ToVector() << endl;
    ackermann_msgs::AckermannDriveStamped drive_msg;
    if (firstScanEstimate){
        std::thread mpc_thread(&MPC::Update, &mpc_, std::ref(current_state), std::ref(trajp_.best_cmaes_trajectory_));//(mpc_.Update(current_state, trajp_.best_cmaes_trajectory_);
        mpc_thread.join();

        inputs_idx_ = 0;
        current_inputs_ = mpc_.get_solved_trajectory();

        mpc_.Visualize();
    }
}

void HRHC::drive_loop()
{
    while(true) {
        if (firstPoseEstimate && firstScanEstimate) {
            ackermann_msgs::AckermannDriveStamped drive_msg;
            Input input = get_next_input();
            drive_msg.drive.speed = input.v();
            drive_msg.drive.steering_angle = input.steer_ang();
            drive_pub_.publish(drive_msg);
            int dt_ms = mpc_.get_dt()*100;
            std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
        }
    }
}

Input HRHC::get_next_input()
{
    if (inputs_idx_ >= current_inputs_.size()) {
        ROS_ERROR("Trajectory complete!");
        return Input(0,0);
    }
    int current_idx = inputs_idx_++;
    return current_inputs_[current_idx];
}

void HRHC::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (firstPoseEstimate)
    {   
        if (!firstScanEstimate)
            firstScanEstimate = true;
        
        occ_grid_.FillOccGrid(current_pose_, scan_msg);
        occ_grid_.Visualize();
        mpc_.update_scan(scan_msg);
        
    }
}

void HRHC::nav_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &nav_goal)
{
    float desired_angle = atan2(2 * nav_goal->pose.orientation.w * nav_goal->pose.orientation.z, 1 - 2 * nav_goal->pose.orientation.z * nav_goal->pose.orientation.z);
    mpc_des_state_.SetX(nav_goal->pose.position.x);
    mpc_des_state_.SetY(nav_goal->pose.position.y);
    mpc_des_state_.SetOri(desired_angle);
}

