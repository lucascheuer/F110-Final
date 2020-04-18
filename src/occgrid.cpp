#include "occgrid.hpp"

OccGrid::OccGrid(int size): size_(size)
{
    ROS_INFO("occgrid created");
}

OccGrid::~OccGrid()
{
    ROS_INFO("killing the occgrid");
}

std::pair<int, int> OccGrid::GridPoint(float x, float y)
{
    return std::pair<int, int>(0, 0);
}