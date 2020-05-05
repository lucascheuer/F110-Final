
#include "pure_pursuit.hpp"
#include "transforms.hpp"
#include "input.hpp"
using namespace std;

vector<float> PurePursuit::getWaypointDistances(geometry_msgs::TransformStamped transform_msg)
{
    vector<float> distances;
    for (auto itr = waypoints_.begin(); itr != waypoints_.end(); itr++) {
        pair<float, float> point = *itr;
        pair<float, float> transformedPoint = Transforms::TransformPoint(point, transform_msg);
        float x = transformedPoint.first;
        float y = transformedPoint.second;
        if (x>0) {
            distances.push_back(sqrt(x*x+y*y));
        } else {
            distances.push_back(numeric_limits<float>::max());
        }
    }
    return distances;
}
PurePursuit::PurePursuit(float lookahead) : lookahead_(lookahead), lastWaypointIndex_(-1)
{}
PurePursuit::~PurePursuit()
{}
void PurePursuit::setCMA_ES(vector<pair<float,float>> &waypoints)
{
    waypoints_ = waypoints;
}
void PurePursuit::getNextWaypoint(const geometry_msgs::Pose &pose, pair<float, float> &carFrameTarget, pair<float,float> &globalFrameTarget)
{
    geometry_msgs::TransformStamped world_to_car_msg_stamped = Transforms::WorldToCarTransform(pose);
    vector<float> distances = getWaypointDistances(world_to_car_msg_stamped);
    float minDistance = numeric_limits<float>::max();
    float argminDist = -1;
    for (int i = 0; i < distances.size(); i++) {
        float currentDistance = distances[i] - lookahead_;
        if (currentDistance >= 0 && currentDistance < minDistance && currentDistance < 1000){ //&& i >= lastWaypointIndex_ || (2*i/waypoints_.size()<2*lastWaypointIndex_/waypoints_.size())) {
            argminDist = i;
            minDistance = currentDistance;
        }
    }
    // no waypoint to follow lah! need to return error
    if (argminDist == -1) {
        ROS_ERROR("Waypoint Cannot");
    }
    lastWaypointIndex_ = argminDist;
    globalFrameTarget = waypoints_[argminDist];

    float x = distances[argminDist];
    // TODO: transform goal point to vehicle frame of reference
    geometry_msgs::Vector3 targetWaypointVector;
    geometry_msgs::Vector3 transformedWaypointVector;
    targetWaypointVector.x = globalFrameTarget.first;
    targetWaypointVector.y = globalFrameTarget.second;
    targetWaypointVector.z = 0;
    tf2::doTransform(targetWaypointVector, transformedWaypointVector, world_to_car_msg_stamped);
    transformedWaypointVector.x += world_to_car_msg_stamped.transform.translation.x;
    transformedWaypointVector.y += world_to_car_msg_stamped.transform.translation.y;
    carFrameTarget = pair<float, float>(transformedWaypointVector.x, transformedWaypointVector.y);
}
