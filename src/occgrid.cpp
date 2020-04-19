#include "occgrid.hpp"
#include <cmath>

OccGrid::OccGrid(int size, float discrete): size_(size),discrete_(discrete)
{
    ROS_INFO("occgrid created");
    grid_blocks_ = size_/discrete_;
    grid_.resize(grid_blocks_, grid_blocks_);
    grid_ = Eigen::MatrixXf::Zero(grid_blocks_, grid_blocks_);
    
}
OccGrid::~OccGrid()
{
    ROS_INFO("killing the occgrid");
}

std::pair<int, int> OccGrid::GridPoint(float x, float y)
{
    int occ_col = (x-occ_offset_.first) / discrete_ + grid_blocks_ / 2;
    int occ_row = (y-occ_offset_.second) / discrete_ + grid_blocks_ / 2;
    return std::pair<int, int>(occ_row,occ_col);
}


std::pair<float,float> OccGrid::GetWorldPoint(int row, int col)
{
    float x = discrete_*(col-grid_blocks_/2)+occ_offset_.first;
    float y = discrete_*(row-grid_blocks_/2)+occ_offset_.second;
    return std::pair<float,float>(x,y);
}

std::pair<float, float> OccGrid::PolarToCartesian(float range, float angle){
    std::pair<float, float> cartesian;
    cartesian.first = range * cos(angle);
    cartesian.second = range * sin(angle);
    return cartesian;
}

void OccGrid::CartesianToOccupancy(float x, float y)
{

    int occ_col = x / discrete_ + grid_blocks_ / 2;
    int occ_row = y / discrete_ + grid_blocks_ / 2;
    if (occ_row >= 0 && occ_row < grid_.rows() && occ_col >= 0 && occ_col < grid_.cols())
    {
        grid_(occ_row, occ_col) = 1;
    }
}

void OccGrid::FillOccGrid(const geometry_msgs::Pose &current_pose,const sensor_msgs::LaserScan::ConstPtr &scan_msg, float obstacle_dilation)
{
    float current_angle = atan2(2 * current_pose.orientation.w * current_pose.orientation.z, 1 - 2 * current_pose.orientation.z * current_pose.orientation.z);
        occ_offset_.first = current_pose.position.x + 0.275 * cos(current_angle);
        occ_offset_.second = current_pose.position.y + 0.275 * sin(current_angle);
        int num_scans = (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment + 1;
        for (int ii = 0; ii < num_scans; ++ii)
        {   
            float angle = scan_msg->angle_min + ii * scan_msg->angle_increment + current_angle;
            std::pair<float, float> cartesian = PolarToCartesian(scan_msg->ranges[ii], angle);

            for (float x_off = -obstacle_dilation; x_off <= obstacle_dilation; x_off += discrete_)
            {
                for (float y_off = -obstacle_dilation; y_off <= obstacle_dilation; y_off += discrete_)
                {
                    //cartesian = polar_to_cartesian(scan_msg->ranges[ii] + jj, angle);
                    CartesianToOccupancy(cartesian.first + x_off, cartesian.second + y_off);
                }
            }
        }
}

bool OccGrid::CartesianInGrid(float x, float y)
{
    std::pair<int, int> grid_point = GridPoint(x, y);
    if (grid_point.first  >= grid_blocks_ || grid_point.first  < 0 ||
        grid_point.second >= grid_blocks_ || grid_point.second < 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}
bool OccGrid::CartesianInGrid(std::pair<float, float> grid_point)
{
    return CartesianInGrid(grid_point.first, grid_point.second);
}
bool OccGrid::CheckCollision(float x1, float y1, float x2, float y2)
{
    return CheckCollision(std::pair<float, float>(x1, y1), std::pair<float, float>(x2, y2));
}

bool OccGrid::CheckCollision(std::pair<float, float> first_point, std::pair<float, float> second_point)
{
    if (!(CartesianInGrid(first_point) &&
          CartesianInGrid(second_point)))
    {
            ROS_ERROR("Out of grid!");
            return false;
    }
    int start_x = first_point.first;
    int start_y = first_point.second;
    int end_x = second_point.first;
    int end_y = second_point.second;
    std::swap(start_x, start_y);
    std::swap(end_x,end_y);
    std::vector<std::pair<float,float>> linePoints;
    if (start_x > end_x)
    {
        std::swap(start_x, end_x);
        std::swap(start_y, end_y);
    }
    int dx = end_x - start_x;
    int dy = end_y - start_y;

    if (std::abs(dy) > std::abs(dx))
    {
        if (dy > 0)
        { // When line has m>1 && m<infinity
            int p = -2*dx + dy; // Initial delta
            int northDelta = -2*dx;
            int northEastDelta = 2*dy - 2*dx;
            for (int x = start_x, y = start_y; y<= end_y; y++)
            {
                if (p > 0)
                {
                    p = p + northDelta;
                } else {
                    p = p + northEastDelta;
                    x++;
                }
                if (grid_(x,y))
                {
                    //line_pub.publish(gen_path_marker(linePoints,0,0,1));
                    return false;
                }
                linePoints.push_back(GetWorldPoint(x,y));
            }
        } else { // When it spills over to second quadrant, but still has abs(m) > 1
            int p = 2*dx - dy; // Initial delta
            int southDelta = 2*dx;
            int southEastDelta = 2*(dy + dx);
            for (int x = start_x, y = start_y; y >= end_y; y--)
            {
                if (p < 0)
                {
                    p = p + southDelta;
                } else {
                    p = p + southEastDelta;
                    x++;
                }
                if (grid_(x,y))
                {
                    //line_pub.publish(gen_path_marker(linePoints,0,0,1));
                    return false;
                }
                linePoints.push_back(GetWorldPoint(x,y));
            }
        }
    } else {
        if (dy > 0)
        {
            int p = 2*dy - dx;
            int eastDelta = 2*dy;
            int northEastDelta = 2*(dy - dx);
            for (int x = start_x, y = start_y; x<= end_x; x++)
            {
                if (p < 0)
                {
                    p = p + eastDelta;
                } else {
                    p = p + northEastDelta;
                    y++;
                }
                if (grid_(x,y))
                {
                    //line_pub.publish(gen_path_marker(linePoints,0,0,1));
                    return false;
                }
                linePoints.push_back(GetWorldPoint(x,y));
            }
        } else {
            int p = 2*dy + dx; // Initial delta
            int eastDelta = 2*dy;
            int southEastDelta = 2*(dy + dx);
            for (int x = start_x, y = start_y; x<= end_x; x++)
            {
                if (p > 0)
                {
                    p = p + eastDelta;
                } else {
                    p = p + southEastDelta;
                    y--;
                }
                if (grid_(x,y))
                {
                    // line_pub.publish(gen_path_marker(linePoints,0,0,1));
                    return false;
                }
                linePoints.push_back(GetWorldPoint(x,y));
            }
        }
    }
    // line_pub.publish(gen_path_marker(linePoints,0,0,1));
    return true;
}


int OccGrid::size()
{
    return size_;
}
