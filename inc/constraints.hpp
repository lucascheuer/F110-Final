#include <ros/ros.h>
#include <Eigen/Geometry>
#include "state.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include "visualizer.hpp"


class Constraints
{
    public:
        Constraints(ros::NodeHandle &nh);
        virtual ~Constraints();
        Eigen::VectorXd x_max();
        Eigen::VectorXd u_max();
        Eigen::VectorXd x_min();
        Eigen::VectorXd u_min();

        Eigen::VectorXd MovedXMax();
        Eigen::VectorXd MovedXMin();

        Eigen::VectorXd l1();
        Eigen::VectorXd l2();
        
        void set_x_max(Eigen::VectorXd xmax);
        void set_u_max(Eigen::VectorXd umax);
        void set_x_min(Eigen::VectorXd xmin);
        void set_u_min(Eigen::VectorXd umin);
        void set_state(State &state);
        void SetXLims(State x); // sets xmax, xmin at +-d
        
        // Eigen::VectorXd
        // void setUlims(float x,float y); 
        void find_half_spaces(State &state,sensor_msgs::LaserScan &scan_msg_);
        

    private:
        Eigen::VectorXd x_max_;
        Eigen::VectorXd u_max_;
        Eigen::VectorXd x_min_;
        Eigen::VectorXd u_min_;
        Eigen::VectorXd l1_;
        Eigen::VectorXd l2_;

        State state_;
        float d;
        float thres_;
        std::pair <float,float> P1_;
        std::pair <float,float> P2_;
        std::pair <float,float> P_;
        std::pair <float,float> P_test_;
        sensor_msgs::LaserScan scan_msg_;
        ros::Publisher points_pub_;
        // ros::Subscriber scan_sub_;
        // void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
        // mode
};