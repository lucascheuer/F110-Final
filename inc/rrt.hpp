
#ifndef RRT_H
#define RRT_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <functional>
#include <cmath>
#include <stdio.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "xtensor/xbuilder.hpp"
#include "xtensor/xtensor.hpp"
#include "xtensor-interpolate/xinterpolate.hpp"
#include "xtensor/xadapt.hpp"
#include "state.hpp"
// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>
#include <utility>
#include "occgrid.hpp"
#include "transforms.hpp"
#include "input.hpp"
// other
#include <Eigen/Geometry>

using namespace std;


/***************************************************************************
 *   THIS CLASS IS DEPRECATED!! IT IS NOT USED IN OUR SUBMISSION!!         *
 *   Included for completeness ONLY                                        *
 ***************************************************************************/

struct Node {
    double x, y;
    int parent; // index of parent node in the tree vector
    bool is_root = false;
    Node()
    {
    }
    Node(double _x, double _y, bool root) 
    {
        x = _x;
        y = _y;
        is_root = root;
    }
};

class RRT {
public:
    RRT(ros::NodeHandle &nh, OccGrid occ_grid);
    virtual ~RRT();
    void updateRRT(geometry_msgs::Pose &pose_update, OccGrid& occ_grid, std::pair<float, float> carFramePoint, std::pair<float, float> globalFramePoint);
    vector<State> getRRTStates(float dt, int horizon);
private:
    float max_angle;
    float steer_angle;
    int rrt_iters;
    float goal_distance;
    float max_goal_distance;
    float min_goal_distance;
    float goal_epsilon;
    float rrt_radius_sq;
    float angle_limit;
    OccGrid occ_grid_;
    geometry_msgs::Pose current_pose_;

    ros::NodeHandle nh_;
    ros::Publisher vis_pub_mult;
    ros::Publisher vis_pub;
    ros::Publisher goal_pub;
    ros::Publisher line_pub;
    ros::Publisher shortcut_pub;
    ros::Publisher tree_pub;

    // TODO: create RRT params

    // random generator, use this
    double max_expansion_dist;
    int shortcut_iters;
    std::mt19937 gen;
    std::uniform_real_distribution<double> x_dist;
    std::uniform_real_distribution<double> y_dist;

    std::vector<pair<float,float>> rrtPath;

    std::vector<Node> tree;

    // callbacks
    int build_tree(std::pair<float, float> targetWaypoint, std::pair<float, float> targetWaypointGlobalCoords);

    // RRT methods
    vector<pair<float,float>> smooth_path(vector<pair<float,float>> path, int discrete);
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node a, Node b);
    bool check_collision(pair<float,float> a, pair<float,float> b);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<pair<float,float>> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);
    double angle_cost(std::vector<Node> &tree, Node &node, Node &p);
    double angle_cost(Node &node, vector<double> &p);
    bool isPathCollisionFree();
    bool isPathCollisionFree(vector<pair<float,float>> &path);
    float getPathLength(vector<pair<float,float>> &path);
    vector<pair<float,float>> shortcutPath(vector<pair<float,float>> path);
    float get_distance(pair<float, float> p1, pair<float, float> p2);

    // 
    void publish_marker(float x, float y);
    visualization_msgs::MarkerArray gen_markers(const vector<pair<float,float>> &points, float r=0, float g=1, float b=0);
    visualization_msgs::Marker gen_path_marker(const vector<pair<float,float>> &path, float r=1, float g=0, float b=0);
    visualization_msgs::MarkerArray gen_RRT_markers();
    visualization_msgs::Marker gen_tree_marker(const vector<Node> &tree, float r=0, float g=1, float b=1);
    vector<geometry_msgs::Point> in_order_path(const vector<pair<float,float>> &path);
    vector<geometry_msgs::Point> full_tree(const vector<Node> &tree);
};

#endif