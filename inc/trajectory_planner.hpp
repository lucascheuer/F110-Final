#ifndef TRAJP_H
#define TRAJP_H
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "occgrid.hpp"
#include "visualizer.hpp"
#include "state.hpp"
using namespace std;

class TrajectoryPlanner
{
    public:
        TrajectoryPlanner();
        TrajectoryPlanner(ros::NodeHandle &nh);
        virtual ~TrajectoryPlanner();
        void readTrajectories();
        void readCMA_ES();
        pair<float,float> carPoint2miniWorld(float x, float y, const geometry_msgs::Pose &current_pose);
        pair<float,float> carPoint2World(float x, float y, const geometry_msgs::Pose &current_pose);
        void trajectory2miniworld(const geometry_msgs::Pose &current_pose);
        void trajectory2world(const geometry_msgs::Pose &current_pose);
        vector<pair<float,float>> trajectories_mini_world;
        vector<pair<float,float>> trajectories_world;
        int best_traj(OccGrid &occ_grid,const geometry_msgs::Pose &current_pose);
        vector<pair<float,float>> cmaes_traj;
        // void visualizeCmaes();
        void Visualize();
        void Update(const geometry_msgs::Pose &pose_msg, OccGrid &occ_grid);
        State best_cmaes_point_;
        vector<State> best_cmaes_trajectory_;
        int num_traj;
        int MAX_HORIZON;
        float close_weight;
        int best_traj_index;

    private:
        
        int horizon_;
        bool successfulRead_;
        void publish_cmaes_closest_marker(float x, float y);
        vector<pair<float,float>> trajectories_;
        float calcDist(pair<float,float> &p1, pair<float,float> &p2);
        pair<float,float> findClosest(pair<float,float> &p1);
        // mode

        // ros viz stuff
        ros::Publisher traj_pub_;
        std::vector<geometry_msgs::Point> points_;
        std::vector<std_msgs::ColorRGBA> colors_;
        bool cmaes_point_pushed_ = false;
        bool cmaes_pushed_ = false;
        bool best_traj_pushed_ = false;

};

#endif
