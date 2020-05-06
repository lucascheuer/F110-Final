#ifndef TRAJP_H
#define TRAJP_H
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "transforms.hpp"
#include "occgrid.hpp"
#include "visualizer.hpp"
#include "state.hpp"
#include "pure_pursuit.hpp"
using namespace std;

// Plans best local path from preset paths for the global paths 
// maximizing progress, and preventing collision and reversal

class TrajectoryPlanner
{
public:
    TrajectoryPlanner();
    // Initializes various parameters from yaml as well as private members
    TrajectoryPlanner(ros::NodeHandle &nh);
    virtual ~TrajectoryPlanner();
    // Reads the set of pre calculated trajectories
    void readTrajectories();
    // Transform a given point from car frame to occupancy-grid frame
    pair<float,float> carPoint2miniWorld(float x, float y, const geometry_msgs::Pose &current_pose);
    // Transforms a given point from car frame to world/map frame
    pair<float,float> carPoint2World(float x, float y, const geometry_msgs::Pose &current_pose);
    // Transforms trajectory from car frame to occupancy grid frame
    void trajectory2miniworld(const geometry_msgs::Pose &current_pose);
    // Transforms trajectory from car frame to world/map frame
    void trajectory2world(const geometry_msgs::Pose &current_pose);
    vector<pair<float,float>> trajectories_mini_world;
    vector<pair<float,float>> trajectories_world;
    // In any given situation, selects the best trajectory 
    // (close to global path + farthest from car + non-reversing) 
    int best_traj(OccGrid &occ_grid,const geometry_msgs::Pose &current_pose);
    // Visualization of local trajectory planned with the aimed goal point
    void Visualize();
    // Lane selection and setting of the best trajectory 
    void Update(const geometry_msgs::Pose &pose_msg, OccGrid &occ_grid);

    State best_cmaes_point_;
    vector<State> best_minipath;
    int num_traj;
    int MAX_HORIZON;
    float close_weight;
    int best_traj_index;

private:
    unsigned int current_lane_ = 0;
    // PurePursuit pure_pursuit;
    std::vector<PurePursuit> lanes_;
    double distance_from_switch_;
    double switch_distance_threshold_;
    // Selects lane based on opponent's position to prevent 
    // collision and promote over-taking
    void SelectLane(const geometry_msgs::Pose pose, OccGrid &occ_grid);
    int horizon_;
    void publish_cmaes_closest_marker(float x, float y);
    vector<pair<float,float>> trajectories_;
    geometry_msgs::Pose last_pose_;

    ros::Publisher traj_pub_;
    std::vector<geometry_msgs::Point> points_;
    std::vector<std_msgs::ColorRGBA> colors_;
    bool cmaes_point_pushed_ = false;
    bool cmaes_pushed_ = false;
    bool best_traj_pushed_ = false;

};

#endif
