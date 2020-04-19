#include "trajectory_planner.hpp"
#include <experimental/filesystem>
#include <fstream>
#include <cmath>
#include <ros/package.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
namespace fs = experimental::filesystem;
int num_traj = 10;
int len_traj = 10;
float BIG_FLOAT = 999999.0f;
TrajectoryPlanner::TrajectoryPlanner()
{
    successfulRead = false;
    ROS_INFO("planner created");
}

TrajectoryPlanner::~TrajectoryPlanner()
{
    ROS_INFO("killing the planner");
}

void TrajectoryPlanner::getTrajectories()
{
    // string path = ros::package::getPath("milestone-3")+"/local_traj.csv";
    string path = "/home/saumya/team3_ws/src/F110-Final/local_traj.csv";
    cout << path << endl;
    ifstream input(path);
    string coordX, coordY;
    if (input.is_open()) {
        while(getline(input,coordX,',')) {
            getline(input,coordY);
            trajectories.push_back(pair<float,float>(stof(coordY),stof(coordX)));
        }
        cout<<"got trajectories \n";
        cout<<trajectories.size();
    // each trajectory has 10 pairs of points, total 100 pairs are present for 10 trajectories
    } else {
        cout << "Please run this from the root catkin_ws directory" << endl;
        exit(0);
    }
}

void TrajectoryPlanner::getCmaes()
{
    // string path = ros::package::getPath("milestone-3")+"/fooxx.csv";
    string path = "/home/saumya/team3_ws/src/F110-Final/fooxx.csv";
    cout << path << endl;
    ifstream input(path);
    string coordX, coordY;
    if (input.is_open()) {
        while(getline(input,coordX,',')) {
            getline(input,coordY);
            cmaes_traj.push_back(pair<float,float>(stof(coordY),stof(coordX)));
        }
        // cout<<"got trajectories";
        // cout<<cmaes_traj.size();
    // each trajectory has 10 pairs of points, total 100 pairs are present for 10 trajectories
    } else {
        cout << "Please run this from the root catkin_ws directory" << endl;
        exit(0);
    }
}

pair<float,float> TrajectoryPlanner::carPoint2miniWorld(float x, float y, const geometry_msgs::Pose &current_pose)
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
    float carPoseX = current_pose.position.x;
    float carPoseY = current_pose.position.y;
    return pair<float,float>(worldPoint.x, worldPoint.y);
}

pair<float,float> TrajectoryPlanner::carPoint2World(float x, float y, const geometry_msgs::Pose &current_pose)
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
    float carPoseX = current_pose.position.x;
    float carPoseY = current_pose.position.y;
    return pair<float,float>(worldPoint.x+carPoseX, worldPoint.y+carPoseY);
}

void TrajectoryPlanner::trajectory2miniworld(const geometry_msgs::Pose &current_pose)
{   
    trajectories_mini_world.clear();;
    for (int i=0; i<trajectories.size();i++)
    {
        trajectories_mini_world.push_back(carPoint2miniWorld(trajectories[i].first, trajectories[i].second,current_pose));
    }
    // ROS_INFO("trajectories in world frame");
}
void TrajectoryPlanner::trajectory2world(const geometry_msgs::Pose &current_pose)
{   
    trajectories_world.clear();;
    for (int i=0; i<trajectories.size();i++)
    {
        trajectories_world.push_back(carPoint2World(trajectories[i].first, trajectories[i].second,current_pose));
    }
    // ROS_INFO("trajectories in world frame");
}
visualization_msgs::MarkerArray TrajectoryPlanner::gen_markers(const vector<pair<float,float>> &points, float r, float g, float b)
{
    visualization_msgs::MarkerArray markerArray;
    for (int ii = 0; ii < points.size(); ii += 1)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = ii;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = points[ii].first;
        marker.pose.position.y = points[ii].second;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        markerArray.markers.push_back(marker);
    }
    return markerArray;
}

float TrajectoryPlanner::calcDist(pair<float,float> &p1, pair<float,float> &p2)
{
    float dist = sqrt(pow((p1.first - p2.first),2) + pow((p1.second-p2.second),2));
    return dist;
}

pair<float,float> TrajectoryPlanner::findClosest(pair<float,float> &p1)
{   pair<float,float> closest;
    float min_dist = BIG_FLOAT;
    for (int i=0; i < cmaes_traj.size();i++)
    {
        if (calcDist(p1,cmaes_traj[i])<min_dist)
        {
            closest = cmaes_traj[i];
            min_dist = calcDist(p1,cmaes_traj[i]);
        }
    }
    return closest;
}

int TrajectoryPlanner::best_traj(OccGrid &occ_grid, const geometry_msgs::Pose &current_pose)
{   
    float min_distance = BIG_FLOAT;
    int best = -1;
    pair<float,float> car_pose (current_pose.position.x,current_pose.position.y);
    for (int ii= 0;ii<num_traj;ii++)
    {   bool collision = false;
        for (int l=0; l<len_traj-1;l++)
        {   
            collision = occ_grid.CheckCollision(trajectories_world[10*ii+l],trajectories_world[10*ii+l+1]);
            if (collision)
            {
                cout<<ii<<" collision"<<endl;
                break;
            }
        }
        if (!collision)
        {
            cout<<ii<<" no_collision"<<endl;
            if (calcDist(car_pose,trajectories_world[10*ii + len_traj-1])<min_distance)
            {
                pair<float,float> end_point = trajectories_world[10*ii + len_traj-1];
                pair<float,float> temp = findClosest(end_point);
                min_distance = calcDist(car_pose,temp);
                best = ii;
            }
        }
    }
    cout<<best<<endl;
    return best;
}


