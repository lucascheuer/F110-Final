#include "occgrid.hpp"

OccGrid::OccGrid(int size, float discrete): size_(size),discrete_(discrete)
{
    ROS_INFO("occgrid created");
    grid_blocks = size_/discrete_;
    grid_ = Eigen::MatrixXf::Zero(grid_blocks,grid_blocks);
    
}
OccGrid::~OccGrid()
{
    ROS_INFO("killing the occgrid");
}


std::pair<int, int> OccGrid::GridPoint(float x, float y)
{
    int occ_col = (x-occ_offset.first) / discrete_ + grid_blocks / 2;
    int occ_row = (y-occ_offset.second) / discrete_ + grid_blocks / 2;
    return std::pair<int, int>(occ_row,occ_col);
}

void fill_occgrid(const geometry_msgs::Pose::ConstPtr &current_pose,const sensor_msgs::LaserScan::ConstPtr& &scan_msg)
{

}


