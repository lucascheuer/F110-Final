#ifndef OCC_H
#define OCC_H
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>

class OccGrid
{
    public:
        OccGrid(int size,float discrete);
        virtual ~OccGrid();
        void FillOccGrid(const geometry_msgs::Pose &pose_msg,const sensor_msgs::LaserScan::ConstPtr& scan_msg, float obstacle_dilation);
        std::pair<int, int> GridPoint(float x, float y);
        std::pair<float, float> PolarToCartesian(float range, float angle);
        void CartesianToOccupancy(float x, float y);
        bool CartesianInGrid(float x, float y);
        std::pair<float,float> GetWorldPoint(int row, int col);

        // accessors
        int size();
    private:
        int size_;
        float discrete_;
        int grid_blocks_;
        std::pair<float, float> occ_offset_;
        Eigen::MatrixXf grid_;

        
        
        
};
#endif