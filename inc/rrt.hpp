#ifndef RRT_H
#define RRT_H
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>

#include <random>

#include "occgrid.hpp"
#include "transforms.hpp"

class RRT
{
    public:
        RRT(OccGrid occ_grid);
        virtual ~RRT();

        void RunRRT(const geometry_msgs::Pose::ConstPtr &pose_update, OccGrid& occ_grid, std::pair<float, float> targetWaypoint, std::pair<float, float> targetWaypointGlobalCoords);

    private:
        
        typedef struct Node {
            double x_, y_;
            int parent_; // index of parent node in the tree vector
            bool is_root_ = false;
            Node()
            {
            }
            Node(double x, double y, bool root) 
            {
                x_ = x;
                y_ = y;
                is_root_ = root;
            }
        } Node;

        // private variables
        int rrt_iters_;
        std::vector<Node> tree_;
        int short_circuit_size_;
        float angle_limit_;
        double max_expansion_dist_;
        float goal_epsilon_;
        geometry_msgs::Pose current_pose_;

        std::vector<std::pair<float,float>> rrt_path_;

        std::mt19937 gen_;
        std::uniform_real_distribution<double> x_distribution_;
        std::uniform_real_distribution<double> y_distribution_;

        // private objects
        OccGrid occ_grid_;

        // private function prototypes
        int BuildTree(std::pair<float, float> targetWaypoint, std::pair<float, float> targetWaypointGlobalCoords);
        std::vector<double> Sample();
        int Nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
        Node Steer(Node &nearest_node, std::vector<double> &sampled_point);
        bool IsGoal(Node &latest_added_node, double goal_x, double goal_y);
        std::vector<std::pair<float,float>> FindPath(std::vector<Node> &tree, Node &latest_added_node);
        double Cost(std::vector<Node> &tree, Node &node);
        double LineCost(Node &n1, Node &n2);
        std::vector<int> Near(std::vector<Node> &tree, Node &node);
        double AngleCost(std::vector<Node> &tree, Node &node, Node &p);
        // mode
};
#endif