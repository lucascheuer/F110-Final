#include "transforms.hpp"

std::pair<float, float> Transforms::CarPointToWorldPoint(float x, float y, geometry_msgs::Pose::ConstPtr &current_pose)
{
    tf2::Transform car_to_world;
    geometry_msgs::Transform car_to_world_msg;
    geometry_msgs::TransformStamped car_to_world_stamped;
    tf2::fromMsg(current_pose, car_to_world);
    car_to_world_msg = tf2::toMsg(car_to_world);
    car_to_world_stamped.transform = car_to_world_msg;
    geometry_msgs::Vector3 carPoint;
    geometry_msgs::Vector3 worldPoint;
    carPoint.x = x;
    carPoint.y = y;
    carPoint.z = 0;
    tf2::doTransform(carPoint, worldPoint, car_to_world_stamped);
    float carPoseX = current_pose->position.x;
    float carPoseY = current_pose->position.y;
    return std::pair<float,float>(worldPoint.x+carPoseX, worldPoint.y+carPoseY);
}