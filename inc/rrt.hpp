// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
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
// other
#include <Eigen/Geometry>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
using namespace std;

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

// TODO: Make occgrid member of class as a 2D boolean/eigen

class RRT {
public:
        RRT(OccGrid occ_grid);
        virtual ~RRT();
private:
    enum MODE {
        RRT_EXPLORE,
        RRT_FOLLOW
    };
    MODE current_mode;
    float max_angle;
    float steer_angle;
    int rrt_iters;
    float goal_distance;
    float max_goal_distance;
    float min_goal_distance;
    float goal_epsilon;
    float look_ahead;
    float rrt_radius_sq;
    float angle_limit;
    OccGrid occ_grid_;
    geometry_msgs::Pose current_pose_;

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
    float explore_dist;
    std::mt19937 gen;
    std::uniform_real_distribution<double> x_dist;
    std::uniform_real_distribution<double> y_dist;

    std::vector<pair<float,float>> rrtPath;

    std::vector<Node> tree;

    void updateRRT(const geometry_msgs::Pose::ConstPtr &pose_update, OccGrid& occ_grid, std::pair<float, float> targetWaypoint, std::pair<float, float> targetWaypointGlobalCoords);
    // callbacks
    // where rrt actually happens
    void pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    int build_tree();

    // RRT methods
    vector<pair<float,float>> smooth_path(vector<pair<float,float>> path);
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

