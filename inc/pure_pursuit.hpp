#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "transforms.hpp"
#include "input.hpp"
#include "state.hpp"
#include "transforms.hpp"
#include "input.hpp"
#include "occgrid.hpp"
#include <ros/package.h>

#include <fstream>
#include <limits>

using namespace std;

class PurePursuit {
private:
    vector<State> waypoints_;
    float lookahead_;
    vector<float> getWaypointDistances(const geometry_msgs::Pose &pose);
public:
    PurePursuit(float lookahead);
    ~PurePursuit();
    bool readCMA_ES(string filename);
    bool isPathCollisionFree(const geometry_msgs::Pose &pose, OccGrid &occ_grid);
    int getClosestIdx(vector<float> &distances, float lookahead);
};
#endif