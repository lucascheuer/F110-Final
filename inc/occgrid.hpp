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
        void fill_occgrid(const geometry_msgs::Pose &pose_msg,const sensor_msgs::LaserScan::ConstPtr& scan_msg, float obstacle_dilation);
        std::pair<int, int> GridPoint(float x, float y);
        int grid_blocks;
        std::pair<float,float> get_world_point(int row, int col);

    private:
        int size_;
        float discrete_;
        Eigen::MatrixXf grid_;
        std::pair<float, float> occ_offset;
        std::pair<float, float> polar_to_cartesian(float range, float angle);
        void cartesian_to_occupancy(float x, float y);

        // mode
};
#endif