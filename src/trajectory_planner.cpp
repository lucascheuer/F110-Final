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
float BIG_FLOAT = 999999.0f;
TrajectoryPlanner::TrajectoryPlanner(ros::NodeHandle &nh)

{   
    int horizon;
    nh.getParam("/horizon", horizon);
    nh.getParam("/num_traj", num_traj);
    nh.getParam("/MAX_HORIZON", MAX_HORIZON);
    nh.getParam("/close_weight", close_weight);
    
    successfulRead_ = false;
    horizon_ = horizon;
    traj_pub_ = nh.advertise<visualization_msgs::Marker>("trajectory_planner", 1);
    ROS_INFO("planner created");
}

TrajectoryPlanner::~TrajectoryPlanner()
{
    ROS_INFO("killing the planner");
}

void TrajectoryPlanner::readTrajectories()
{
    string path = ros::package::getPath("milestone-3")+"/local_traj_50.csv";
    // string path = "/home/saumya/team3_ws/src/F110-Final/local_traj_50.csv";
    cout << path << endl;
    ifstream input(path);
    string coordX, coordY;
    if (input.is_open()) {
        while(getline(input,coordX,',')) {
            getline(input,coordY);
            trajectories_.push_back(pair<float,float>(stof(coordY),stof(coordX)));
        }
        cout<<"got trajectories \n";
        cout<<trajectories_.size();
    // each trajectory has 10 pairs of points, total 100 pairs are present for 10 trajectories
    } else {
        cout << "Please run this from the root catkin_ws directory" << endl;
        exit(0);
    }
}

void TrajectoryPlanner::readCMA_ES()
{
    string path = ros::package::getPath("milestone-3")+"/fooxx.csv";
    // string path = "/home/saumya/team3_ws/src/F110-Final/fooxx.csv";
    cout << path << endl;
    ifstream input(path);
    string coordX, coordY;
    if (input.is_open()) {
        while(getline(input,coordX,',')) {
            getline(input,coordY);
            cmaes_traj.push_back(pair<float,float>(stof(coordX),stof(coordY)));
        }
    }
    else {
        cout << "Please run this from the root catkin_ws directory" << endl;
        exit(0);
    }    
        // cout<<"got trajectories";
        // cout<<cmaes_traj.size();
    // each trajectory has 10 pairs of points, total 100 pairs are present for 10 trajectories
    

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
    for (int i=0; i<trajectories_.size();i++)
    {
        trajectories_mini_world.push_back(carPoint2miniWorld(trajectories_[i].first, trajectories_[i].second,current_pose));
    }
    // ROS_INFO("trajectories in world frame");
}
void TrajectoryPlanner::trajectory2world(const geometry_msgs::Pose &current_pose)
{   
    trajectories_world.clear();
    for (int i=0; i<trajectories_.size();i++)
    {
        trajectories_world.push_back(carPoint2World(trajectories_[i].first, trajectories_[i].second,current_pose));
    }
    // ROS_INFO("trajectories in world frame");
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
    float max_distance = -BIG_FLOAT;
    int best = -1;
    pair<float,float> closest_cmaes;
    pair<float,float> car_pose (current_pose.position.x,current_pose.position.y);
    for (int ii= 0;ii<num_traj;ii++)
    {
        bool collision = true;
        
        for (int l=0; l<horizon_ - 1;l++)
        {   
            collision = occ_grid.CheckCollision(trajectories_world[MAX_HORIZON*ii+l],trajectories_world[MAX_HORIZON*ii+l+1]);
            if (!collision)
            {
                //cout<<ii<<" collision"<<endl;
                break;
            }
            //cout << l << ",";
        }
        // 0 1.44218

                
        if (collision)
        {
            // cout<<ii<<" no_collision"<<endl;
            pair<float,float> end_point = trajectories_world[MAX_HORIZON*ii + horizon_-1];
            pair<float,float> temp = findClosest(end_point);
            float dist1 = calcDist(end_point,temp);
            float dist2 = calcDist(temp,car_pose);
            
            if (dist2-close_weight*dist1>max_distance)
            {
                max_distance = dist2-close_weight*dist1;
                closest_cmaes = temp;
                best = ii;
            }
        }
        
    }
    // cout<<max_distance<<endl;
    // cout<<best<<endl;
    // publish_cmaes_closest_marker(trajectories_world[10 * best + 5].first,trajectories_world[10 * best + 5].second);
    if (!cmaes_point_pushed_)
    {
        geometry_msgs::Point curr_point;
        curr_point.x = closest_cmaes.first;
        curr_point.y = closest_cmaes.second;
        curr_point.z = 0.2;
        points_.push_back(curr_point);
        std_msgs::ColorRGBA curr_color;//(1.0, 0.0, 1.0, 1.0);
        curr_color.r = 0;
        curr_color.g = 0;
        curr_color.b = 1;
        curr_color.a = 1;
        colors_.push_back(curr_color);
        cmaes_point_pushed_ = true;
    }
    // publish_cmaes_closest_marker(closest_cmaes.first,closest_cmaes.second);
    State push;
    push.SetX(trajectories_world[MAX_HORIZON * best].first);
    push.SetY(trajectories_world[MAX_HORIZON * best].second);
    double dx = (trajectories_world[MAX_HORIZON * best + 1].first) - (trajectories_world[MAX_HORIZON * best].first);
    double dy = (trajectories_world[MAX_HORIZON * best + 1].second) - (trajectories_world[MAX_HORIZON * best].second);
    double ori = atan2(dy, dx);
    push.SetOri(ori);
    best_cmaes_trajectory_.clear();
    best_cmaes_trajectory_.push_back(push);
    for (int ii =  1; ii < horizon_; ++ii)
    {
        push.SetX(trajectories_world[MAX_HORIZON * best + ii].first);
        push.SetY(trajectories_world[MAX_HORIZON * best + ii].second);
        dx = (trajectories_world[MAX_HORIZON * best + ii].first) - (trajectories_world[MAX_HORIZON * best + ii - 1].first);
        dy = (trajectories_world[MAX_HORIZON * best + ii].second) - (trajectories_world[MAX_HORIZON * best + ii - 1].second);
        ori = atan2(dy, dx);
        push.SetOri(ori);
        best_cmaes_trajectory_.push_back(push);
    }
    best_cmaes_point_.SetX(trajectories_world[MAX_HORIZON * best + horizon_-1].first);
    best_cmaes_point_.SetY(trajectories_world[MAX_HORIZON * best + horizon_-1].second);
    dx = (trajectories_world[MAX_HORIZON * best + horizon_-1].first) - (trajectories_world[MAX_HORIZON * best + horizon_ - 2].first);
    dy = (trajectories_world[MAX_HORIZON * best + horizon_-1].second) - (trajectories_world[MAX_HORIZON * best + horizon_ - 2].second);
    ori = atan2(dy, dx);
    // cout << dx << "\t" << dy << "\t" << ori << endl;
    best_cmaes_point_.SetOri(ori);
    // cout<<best<<"is the best"<<endl;
    return best;
}


void TrajectoryPlanner::Visualize()
{
    std::vector<pair<float,float>> best_traj;
    for (int i = MAX_HORIZON*best_traj_index; i<MAX_HORIZON*best_traj_index+horizon_;i++)
    {
        best_traj.push_back(trajectories_world[i]);
    }
    std::vector<geometry_msgs::Point> best_traj_points = Visualizer::GenerateVizPoints(best_traj);
    std::vector<std_msgs::ColorRGBA> best_traj_colors = Visualizer::GenerateVizColors(best_traj, 0, 1, 0);
    points_.insert(points_.end(), best_traj_points.begin(), best_traj_points.end());
    colors_.insert(colors_.end(), best_traj_colors.begin(), best_traj_colors.end());
    best_traj_pushed_ = true;
    // best_traj_pub_.publish(Visualizer::GenerateSphereList(best_traj, 0, 1, 0));
    std::vector<geometry_msgs::Point> cmaes_points = Visualizer::GenerateVizPoints(cmaes_traj);
    std::vector<std_msgs::ColorRGBA> cmaes_colors = Visualizer::GenerateVizColors(cmaes_traj, 1, 0, 0);
    points_.insert(points_.end(), cmaes_points.begin(), cmaes_points.end());
    colors_.insert(colors_.end(), cmaes_colors.begin(), cmaes_colors.end());
    cmaes_pushed_ = true;
    traj_pub_.publish(Visualizer::GenerateList(points_, colors_));
    cmaes_point_pushed_ = false;
    best_traj_pushed_ = false;
    cmaes_pushed_ = false;
    points_.clear();
    colors_.clear();
    // visualizeCmaes();
}
void TrajectoryPlanner::Update(const geometry_msgs::Pose &current_pose, OccGrid &occ_grid)
{
    //trajectory2miniworld(current_pose);
    trajectory2world(current_pose);
    best_traj_index = best_traj(occ_grid, current_pose);
    
}
