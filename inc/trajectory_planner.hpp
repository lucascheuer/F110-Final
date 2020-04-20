#ifndef TRAJP_H
#define TRAJP_H
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "occgrid.hpp"
#include "visualizer.hpp"
using namespace std;

class TrajectoryPlanner
{
    public:
        TrajectoryPlanner(ros::NodeHandle &nh);
        virtual ~TrajectoryPlanner();
        void getTrajectories();
        void getCmaes();
        pair<float,float> carPoint2miniWorld(float x, float y, const geometry_msgs::Pose &current_pose);
        pair<float,float> carPoint2World(float x, float y, const geometry_msgs::Pose &current_pose);
        void trajectory2miniworld(const geometry_msgs::Pose &current_pose);
        void trajectory2world(const geometry_msgs::Pose &current_pose);
        vector<pair<float,float>> trajectories_mini_world;
        vector<pair<float,float>> trajectories_world;
        int best_traj(OccGrid &occ_grid,const geometry_msgs::Pose &current_pose);
        vector<pair<float,float>> cmaes_traj;
        void visualizeCmaes();
        void Visualize();
        void Update(const geometry_msgs::Pose &pose_msg, OccGrid &occ_grid);
        

    private:
        ros::Publisher closest_cmaes_pub_;
        ros::Publisher cmaes_pub_;
        ros::Publisher best_traj_pub_;
        int best_traj_index_;
        bool successfulRead_;
        void publish_cmaes_closest_marker(float x, float y);
        vector<pair<float,float>> trajectories_;
        float calcDist(pair<float,float> &p1, pair<float,float> &p2);
        pair<float,float> findClosest(pair<float,float> &p1);
        // mode
};

#endif
