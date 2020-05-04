#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "transforms.hpp"
#include "input.hpp"
using namespace std;

class PurePursuit {
private:
    vector<pair<float,float>> waypoints_;
    int lastWaypointIndex_;
    float lookahead_;
    vector<float> getWaypointDistances(geometry_msgs::TransformStamped transform_msg);
public:
    PurePursuit(float lookahead);
    ~PurePursuit();
    void setCMA_ES(vector<pair<float,float>> &waypoints);
    void getNextWaypoint(const geometry_msgs::Pose &pose, pair<float, float> &carFrameTarget, pair<float,float> &globalFrameTarget);
};
#endif