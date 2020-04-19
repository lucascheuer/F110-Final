#include "rrt.hpp"

RRT::RRT(OccGrid occ_grid): occ_grid_(occ_grid)
{   
    int divide = 3;
    x_distribution_ = uniform_real_distribution<double>(0.4,occ_grid_.size()/divide);
    y_distribution_ = uniform_real_distribution<double>(-occ_grid_.size()/divide,occ_grid_.size()/divide);
    ROS_INFO("rrt created");
}

RRT::~RRT()
{
    ROS_INFO("killing the rrt");
}
void RRT::RunRRT(const geometry_msgs::Pose::ConstPtr &pose_update, OccGrid& occ_grid, std::pair<float, float> targetWaypoint, std::pair<float, float> targetWaypointGlobalCoords)
{
    current_pose_.position = pose_update->position;
    current_pose_.orientation = pose_update->orientation;
    occ_grid_ = occ_grid;
    int index = BuildTree(targetWaypoint, targetWaypointGlobalCoords);
    rrt_path_ = FindPath(tree_, tree_[index]);
}

int RRT::BuildTree(std::pair<float, float> targetWaypoint, std::pair<float, float> targetWaypointGlobalCoords)
{
    float current_angle = atan2(2 * current_pose_.orientation.w * current_pose_.orientation.z, 1 - 2 * current_pose_.orientation.z * current_pose_.orientation.z);
    tree_.clear();
    float posex = current_pose_.position.x + 0.275 * cos(current_angle);
    float posey = current_pose_.position.y + 0.275 * sin(current_angle);
    RRT::Node root(posex, posey, true);
    root.parent_ = -1;
    tree_.push_back(root);
    
    for (int i = 0; i < rrt_iters_; i++)
    {
        // FIXME: i think this should be from 0,0
        std::vector<double> x_rand;        
        RRT::Node x_new;
        int x_nearest_idx;

        x_rand.push_back(targetWaypointGlobalCoords.first);
        x_rand.push_back(targetWaypointGlobalCoords.second);
        x_nearest_idx = Nearest(tree_, x_rand);
        x_new.x_ = x_rand[0];
        x_new.y_ = x_rand[1];
        if (tree_.size()>short_circuit_size_ && occ_grid_.CheckCollision(tree_[x_nearest_idx].x_, tree_[x_nearest_idx].y_, x_new.x_, x_new.y_) && AngleCost(tree_, tree_[x_nearest_idx],x_new)<angle_limit_)
        {
            x_new.parent_ = x_nearest_idx;
            
            //publish_marker(x_new.x, x_new.y);
            tree_.push_back(x_new);
            if (IsGoal(x_new, targetWaypointGlobalCoords.first, targetWaypointGlobalCoords.second))
            {
                // ROS_INFO("breaking out coz of shortcircuit");
                break;
            }
        } else {
            x_rand = Sample();
            x_nearest_idx = Nearest(tree_, x_rand);
            x_new = Steer(tree_[x_nearest_idx], x_rand);
            if (occ_grid_.CheckCollision(tree_[x_nearest_idx].x_, tree_[x_nearest_idx].y_, x_new.x_, x_new.y_) && AngleCost(tree_, tree_[x_nearest_idx],x_new)<angle_limit_)
            {
                std::vector<int> nearby = Near(tree_, x_new);

                int x_min = x_nearest_idx;
                double c_min = Cost(tree_, tree_[x_nearest_idx]) + LineCost(tree_[x_nearest_idx], x_new);
                for (int current_near = 0; current_near < nearby.size(); ++current_near)
                {
                    if (occ_grid_.CheckCollision(tree_[nearby[current_near]].x_, tree_[nearby[current_near]].y_, x_new.x_, x_new.y_) && (Cost(tree_, tree_[nearby[current_near]]) + LineCost(x_new, tree_[nearby[current_near]])<c_min))
                    {
                        x_min = nearby[current_near];
                        c_min = Cost(tree_, tree_[nearby[current_near]]) + LineCost(x_new, tree_[nearby[current_near]]);
                    }
                }
                x_new.parent_ = x_min;
                tree_.push_back(x_new);
                for (int current_near = 0; current_near < nearby.size(); ++current_near)
                {
                    if (occ_grid_.CheckCollision(tree_[nearby[current_near]].x_, tree_[nearby[current_near]].y_, x_new.x_, x_new.y_) && (Cost(tree_, x_new) + LineCost(x_new, tree_[nearby[current_near]])) < Cost(tree_, tree_[nearby[current_near]]))
                    {
                        tree_[nearby[current_near]].parent_ = tree_.size()-1;
                    }
                }
                if (IsGoal(x_new, targetWaypointGlobalCoords.first, targetWaypointGlobalCoords.second))
                {
                    // ROS_INFO("breaking out coz GOAL FOUND");
                    break;
                }
            }
        }
        
        
    }
    
    float minDistance = std::numeric_limits<float>::max();
    int minIndex = 0;
    for (auto itr = tree_.begin(); itr != tree_.end(); itr++)
    {
        std::pair<float,float> point(itr->x_,itr->y_);
        geometry_msgs::TransformStamped transform_msg = Transforms::WorldToCarTransform(current_pose_);
        std::pair<float, float> transformedPoint = Transforms::TransformPoint(point, transform_msg);
        if (transformedPoint.first > 0)
        {
            float distance = sqrt(pow(transformedPoint.first-targetWaypoint.first,2)+pow(transformedPoint.second-targetWaypoint.second,2));
            if (distance < minDistance)
            {
                minDistance = distance;
                minIndex = itr - tree_.begin();
            }
        }
    }
    // FIXME
    // publish_marker(tree_[minIndex].x_, tree_[minIndex].y_);
    return minIndex;
}

std::vector<double> RRT::Sample()
{
    std::vector<double> sampled_point;
    float x = x_distribution_(gen_);
    float y = y_distribution_(gen_);
    std::pair<float,float> newPoint = Transforms::CarPointToWorldPoint(x,y, current_pose_);
    sampled_point.push_back(newPoint.first);
    sampled_point.push_back(newPoint.second);
    return sampled_point;
}
int RRT::Nearest(std::vector<RRT::Node> &tree, std::vector<double> &sampled_point)
{
    float min_dist = 100;
    float curr_dist;
    int nearest_node = 0;
    for (int i = 0;i<tree.size();i++){
        curr_dist = pow((tree_[i]).x_-sampled_point[0],2) + pow((tree_[i]).y_-sampled_point[1],2);
        if (curr_dist<min_dist){
            min_dist = curr_dist;
            nearest_node = i;
        }
    }
    return nearest_node;
}
RRT::Node RRT::Steer(RRT::Node &nearest_node, std::vector<double> &sampled_point)
{
    float curr_distance = sqrt(pow(nearest_node.x_ - sampled_point[0],2) + pow(nearest_node.y_ - sampled_point[1],2));
    RRT::Node new_node;
    if (curr_distance > max_expansion_dist_){
        new_node.x_ = nearest_node.x_ + ((sampled_point[0]-nearest_node.x_)*max_expansion_dist_/curr_distance);
        new_node.y_ = nearest_node.y_ + ((sampled_point[1]-nearest_node.y_)*max_expansion_dist_/curr_distance);
    } else {
        new_node.x_ = sampled_point[0];
        new_node.y_ = sampled_point[1];
    }
    return new_node;
}


bool RRT::IsGoal(RRT::Node &latest_added_node, double goal_x, double goal_y)
{
    bool close_enough = false;
    float curr_dist = sqrt(pow(latest_added_node.x_-goal_x,2) + pow(latest_added_node.y_-goal_y,2));
    if (curr_dist<goal_epsilon_){
        close_enough = true;
    }
    return close_enough;
}
std::vector<std::pair<float,float>> RRT::FindPath(std::vector<RRT::Node> &tree, RRT::Node &latest_added_node)
{
    bool root = false;
    std::vector<std::pair<float,float>> found_path;
    RRT::Node curr_node = latest_added_node;
    std::pair<float,float> nodexy;
    nodexy.first = (float)curr_node.x_;
    nodexy.second= (float)curr_node.y_;
    found_path.push_back(nodexy);
    while(!curr_node.is_root_) {
        curr_node = tree_[curr_node.parent_];
        root = curr_node.is_root_;
        nodexy.first = curr_node.x_;
        nodexy.second = curr_node.y_;
        found_path.push_back(nodexy);
    }
    std::reverse(found_path.begin(),found_path.end());
    return found_path;
}
double RRT::Cost(std::vector<RRT::Node> &tree, RRT::Node &node)
{
    double cost = 0;
    RRT::Node curr_node = node;
    int save = 0;
    while (!curr_node.is_root_)// && save < tree_.size())
    {
        cost += LineCost(curr_node, tree_[curr_node.parent_]);
        // cost += angle_cost(tree_, tree_[curr_node.parent],curr_node);
        curr_node = tree_[curr_node.parent_];
        save = curr_node.parent_;
    }
    return cost;
}
std::vector<int> RRT::Near(std::vector<RRT::Node> &tree, RRT::Node &node)
{
    std::vector<int> neighborhood;
    for (int ii = 0; ii < tree_.size(); ++ii)
    {
        if (LineCost(tree_[ii], node) < std::min(pow(log((double)tree.size()) / tree_.size(), 0.5), max_expansion_dist_))
        {
            neighborhood.push_back(ii);
        }
    }
    return neighborhood;
}

double RRT::LineCost(RRT::Node &n1, RRT::Node &n2)
{
    return sqrt(pow(n1.x_ - n2.x_, 2) + pow(n1.y_ - n2.y_, 2));
}

double RRT::AngleCost(std::vector<RRT::Node> &tree, RRT::Node &node, RRT::Node &p)
{
    double cost = 0;
    double cos_angle;
    if (!node.is_root_)
    {   
        std::pair<float,float> vec1;
        std::pair<float,float> vec2;
        vec1.first = p.x_ - node.x_;
        vec1.second = p.y_ - node.y_;
        RRT::Node par_node = tree[node.parent_];
        vec2.first = node.x_ -par_node.x_;
        vec2.second = node.y_ - par_node.y_;
        cos_angle = (vec1.first*vec2.first + vec1.second*vec2.second)/(sqrt(pow(vec1.first,2)+pow(vec1.second,2))+sqrt(pow(vec2.first,2)+pow(vec2.second,2)));
        if (cos_angle>0)
        {
            cost = 1/cos_angle;
        }
        else
        {
            cost = std::numeric_limits<float>::max()/100;
        }
        // cout<<cost<<'\n';
    }
    return cost;
}