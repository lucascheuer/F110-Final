#ifndef TRANS_H
#define TRANS_H
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
class Transforms
{
    public:
        static std::pair<float, float> CarPointToWorldPoint(float x, float y, geometry_msgs::Pose::ConstPtr &current_pose);
};
#endif