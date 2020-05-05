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
    // Returns distances of waypoints in CMA-ES trajectory relative to
    // car's position
    vector<float> getWaypointDistances(const geometry_msgs::Pose &pose);
public:
    PurePursuit(float lookahead);
    ~PurePursuit();
    // Converts trajectory of State objects to pairs of X,Y coordinates
    vector<pair<float,float>> getPairPoints();
    // Loads CSV of CMA-ES trajectories
    bool readCMA_ES(string filename);
    // Checks if path is collision free till lookahead distance
    bool isPathCollisionFree(const geometry_msgs::Pose pose, OccGrid &occ_grid);
    // Helper function for isPathCollisionFree(..)
    int getClosestIdx(vector<float> &distances, float lookahead);
    // Finds the closest point in the CMA-ES trajectory
    pair<float,float> findClosest(pair<float,float> &globalPoint);
};
#endif