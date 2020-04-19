#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>

class OccGrid
{
    public:
        OccGrid(int size,float discrete);
        virtual ~OccGrid();
        void fill_occgrid(const geometry_msgs::Pose::ConstPtr &pose_msg,const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    private:
        int size_;
        float discrete_;
        int grid_blocks;
        Eigen::MatrixXf grid_;
        std::pair<int, int> GridPoint(float x, float y);
        std::pair<float, float> occ_offset;
        // mode
};