#ifndef OCC_H
#define OCC_H
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

class OccGrid
{
    public:
        OccGrid(ros::NodeHandle &nh, int size,float discrete);
        virtual ~OccGrid();
        std::pair<int, int> WorldToOccupancy(std::pair<float, float> point);
        std::pair<int, int> WorldToOccupancy(float x, float y);
        std::pair<float,float> OccupancyToWorld(int row, int col);
        std::pair<float,float> OccupancyToWorld(std::pair<int,int> grid_point);
        std::pair<float, float> PolarToCartesian(float range, float angle);
        void FillOccGrid(const geometry_msgs::Pose &pose_msg,const sensor_msgs::LaserScan::ConstPtr& scan_msg, float obstacle_dilation);
        bool InGrid(int col, int row);
        bool InGrid(std::pair<int, int> grid_point);
        bool CartesianInGrid(float x, float y);
        bool CartesianInGrid(std::pair<float, float> cart_point);
        bool CheckCollision(float x1, float y1, float x2, float y2);
        bool CheckCollision(std::pair<float, float> first_point, std::pair<float, float> second_point);
        
        void Visualize();


        // accessors
        int size();
    private:
        int size_;
        float discrete_;
        int grid_blocks_;
        std::pair<float, float> occ_offset_;
        Eigen::MatrixXf grid_;
        ros::Publisher occ_pub_;
        
        
        
};
#endif