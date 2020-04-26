#include "constraints.hpp"
#include "visualizer.hpp"
#include <cmath>

Constraints::Constraints(ros::NodeHandle &nh)
{
    std::string hrhc_node = "";
    nh.getParam(hrhc_node+"/umax", umax_val_);
    nh.getParam(hrhc_node+"/umin", umin_val_);
    nh.getParam(hrhc_node+"/follow_gap_thresh", thres_);
    nh.getParam(hrhc_node+"/state_lims", d);
    nh.getParam(hrhc_node+"/fov_divider", divider_);

    x_max_.resize(3,1);
    x_max_ << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY; //x,y,ori
    x_min_.resize(3,1);
    x_min_ << -OsqpEigen::INFTY,-OsqpEigen::INFTY,-OsqpEigen::INFTY; //x,y,ori
    u_max_.resize(2,1);
    u_max_ << umax_val_, 0.43f; //Speed, steering
    u_min_.resize(2,1);
    u_min_ << umin_val_, -0.43f; //Speed, steering
    // d = 1.0f;
    // thres_ = 1.5f;
    points_pub_ = nh.advertise<visualization_msgs::Marker>("triangle_points", 100);
    // scan_sub_ = nh.subscribe("/scan", 10, &Constraints::scan_callback, this);
    
    // ROS_INFO("constraints created");
}

// void Constraints::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
// {
//     scan_msg_ = scan_msg;
//     // scan_msg_.ranges = scan_msg->ranges;
//     // std::cout<<"got rangesss";
//     // scan_msg_.angle_max = scan_msg->angle_max;
//     // scan_msg_.angle_min = scan_msg->angle_min;
//     // scan_msg_.angle_increment = scan_msg->angle_increment;
    
// }


Constraints::~Constraints()
{
    // ROS_INFO("killing the constraints");
}


Eigen::VectorXd Constraints::x_max()
{
    return x_max_;
}

Eigen::VectorXd Constraints::x_min()
{
    return x_min_;
}

Eigen::VectorXd Constraints::u_max()
{
    return u_max_;
}

Eigen::VectorXd Constraints::u_min()
{
    return u_min_;
}

Eigen::VectorXd Constraints::MovedXMax()
{
    return x_max_ += state_.ToVector();
}

Eigen::VectorXd Constraints::MovedXMin()
{
    return x_min_ += state_.ToVector();
}



void Constraints::set_x_max(Eigen::VectorXd xmax)
{
    x_max_.resize(xmax.size());
    x_max_ = xmax;
}

void Constraints::set_u_max(Eigen::VectorXd umax)
{
    u_max_.resize(umax.size());
    u_max_ = umax;
}

Eigen::VectorXd Constraints::l1()
{
    return l1_;
}

Eigen::VectorXd Constraints::l2()
{
    return l2_;
}

void Constraints::set_x_min(Eigen::VectorXd xmin)
{
    x_min_.resize(xmin.size());
    x_min_ = xmin;
}

void Constraints::set_u_min(Eigen::VectorXd umin)
{
    u_min_.resize(umin.size());
    u_min_ = umin;
}

void Constraints::set_state(State &state)
{
    state_ = state;
}

void Constraints::SetXLims(State state)
{
    x_max_(0,0) = state.x()+d;
    x_max_(1,0) = state.y()+d;
    x_min_(0,0) = state.x()-d;
    x_min_(1,0) = state.y()-d;
}

void Constraints::find_half_spaces(State &state,sensor_msgs::LaserScan &scan_msg_)
{
    int num_scans = (scan_msg_.angle_max - scan_msg_.angle_min) / scan_msg_.angle_increment + 1;
    int max_gap = -1;
    int best_lo = 0;
    int best_hi = 0;
    int lo = -1;
    int hi = -1;
    double poseX = state.x();
    double poseY = state.y();
    float current_angle = state.ori();
    bool in_gap = 0;
    for (int ii=0; ii<num_scans; ii++)
    {   float angle = scan_msg_.angle_min + ii * scan_msg_.angle_increment;
        if (angle>-1.571f/divider_ && angle < 1.571f/divider_) // value is pi/2
        {
            if (scan_msg_.ranges[ii] > thres_)
            {   
                if (in_gap)
                {
                    hi = ii;
                }
                else
                {
                    lo = ii;
                    in_gap = 1;
                }
                
            }
            else
            {
                in_gap = 0;
                if (hi-lo>max_gap)
                {
                    max_gap = hi-lo;
                    best_hi = hi;
                    best_lo = lo;
                }
            }
            if (hi-lo>max_gap)
                {
                    max_gap = hi-lo;
                    best_hi = hi;
                    best_lo = lo;
                }
        }
    }
    float angle1 = scan_msg_.angle_min + best_lo * scan_msg_.angle_increment + current_angle;
    float angle2 = scan_msg_.angle_min + best_hi * scan_msg_.angle_increment + current_angle;
    
    P1_.first = scan_msg_.ranges[best_lo] * cos(angle1) + poseX;// - 0.275 * cos(current_angle);
    P1_.second = scan_msg_.ranges[best_lo] * sin(angle1) + poseY;// - 0.275 * sin(current_angle);

    P2_.first = scan_msg_.ranges[best_hi] * cos(angle2) + poseX ;//- 0.275 * cos(current_angle);
    P2_.second = scan_msg_.ranges[best_hi] * sin(angle2) + poseY ;//- 0.275 * sin(current_angle);

    P_.first = poseX ;//- 0.275 * cos(current_angle);
    P_.second = poseY ;//- 0.275 * sin(current_angle);

    std::vector<geometry_msgs::Point> triangle_points;
    geometry_msgs::Point p1;
    p1.x = P1_.first;
    p1.y = P1_.second;
    p1.z = 0.1;
    triangle_points.push_back(p1);

    geometry_msgs::Point p2;
    p2.x = P2_.first;
    p2.y = P2_.second;
    p2.z = 0.1;
    triangle_points.push_back(p2);

    geometry_msgs::Point p;
    p.x = P_.first;
    p.y = P_.second;
    p.z = 0.1;
    triangle_points.push_back(p);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "current";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.points = triangle_points;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 2; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    points_pub_.publish(marker);

    float a1,b1,c1;
    float a2,b2,c2;
    a1 = P_.second - P1_.second;
    b1 = P1_.first - P_.first;
    c1 = P_.first*P1_.second - P_.second*P1_.first;

    if (a1*P2_.first+b1*P2_.second+c1<0)
    {
        a1 = -a1;
        b1 = -b1;
        c1 = -c1;
    }
    
    a2 = P_.second - P2_.second;
    b2 = P2_.first - P_.first;
    c2 = P_.first*P2_.second - P_.second*P2_.first;

    if (a2*P1_.first+b2*P1_.second+c2<0)
    {
        a2 = -a2;
        b2 = -b2;
        c2 = -c2;
    }

    l1_.resize(3);
    l2_.resize(3);
    
    l1_(0) = a1;
    l1_(1) = b1;
    l1_(2) = c1+0.5;
    
    l2_(0) = a2;
    l2_(1) = b2;
    l2_(2) = c2+0.5;
    


}
