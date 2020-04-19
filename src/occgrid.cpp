#include "occgrid.hpp"
#include <cmath>

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
std::pair<float, float> OccGrid::polar_to_cartesian(float range, float angle)
{
    std::pair<float, float> cartesian;
    cartesian.first = range * cos(angle);
    cartesian.second = range * sin(angle);
    return cartesian;
}

void OccGrid::cartesian_to_occupancy(float x, float y)
{

    int occ_col = x / discrete_ + grid_blocks / 2;
    int occ_row = y / discrete_ + grid_blocks / 2;
    if (occ_row >= 0 && occ_row < grid_.rows() && occ_col >= 0 && occ_col < grid_.cols())
    {
        grid_(occ_row, occ_col) = 1;
    }
}

void OccGrid::fill_occgrid(const geometry_msgs::Pose &current_pose,const sensor_msgs::LaserScan::ConstPtr &scan_msg, float obstacle_dilation)
{
    float current_angle = atan2(2 * current_pose.orientation.w * current_pose.orientation.z, 1 - 2 * current_pose.orientation.z * current_pose.orientation.z);
        occ_offset.first = current_pose.position.x + 0.275 * cos(current_angle);
        occ_offset.second = current_pose.position.y + 0.275 * sin(current_angle);
        int num_scans = (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment + 1;
        for (int ii = 0; ii < num_scans; ++ii)
        {   
            float angle = scan_msg->angle_min + ii * scan_msg->angle_increment + current_angle;
            std::pair<float, float> cartesian = polar_to_cartesian(scan_msg->ranges[ii], angle);

            for (float x_off = -obstacle_dilation; x_off <= obstacle_dilation; x_off += discrete_)
            {
                for (float y_off = -obstacle_dilation; y_off <= obstacle_dilation; y_off += discrete_)
                {
                    //cartesian = polar_to_cartesian(scan_msg->ranges[ii] + jj, angle);
                    cartesian_to_occupancy(cartesian.first + x_off, cartesian.second + y_off);
                }
            }
        }
}


