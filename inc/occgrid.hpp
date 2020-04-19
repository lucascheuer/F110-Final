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

    private:
        int size_;
        float discrete_;
        int grid_blocks;
        Eigen::MatrixXf grid_;
        std::pair<int, int> GridPoint(float x, float y);
        std::pair<float, float> occ_offset;
        std::pair<float, float> polar_to_cartesian(float range, float angle);
        void cartesian_to_occupancy(float x, float y);

        // mode
};