#include <ros/ros.h>
#include <Eigen/Geometry>
#include "state.hpp"
#include <OsqpEigen/OsqpEigen.h>


class Constraints
{
    public:
        Constraints();
        virtual ~Constraints();
        Eigen::VectorXd x_max();
        Eigen::VectorXd u_max();
        Eigen::VectorXd x_min();
        Eigen::VectorXd u_min();

        Eigen::VectorXd MovedXMax();
        Eigen::VectorXd MovedXMin();

        void set_x_max(Eigen::VectorXd xmax);
        void set_u_max(Eigen::VectorXd umax);
        void set_x_min(Eigen::VectorXd xmin);
        void set_u_min(Eigen::VectorXd umin);
        void set_state(State state);
        void SetXLims(State x); // sets xmax, xmin at +-d
        // Eigen::VectorXd
        // void setUlims(float x,float y); 

    private:
        Eigen::VectorXd x_max_;
        Eigen::VectorXd u_max_;
        Eigen::VectorXd x_min_;
        Eigen::VectorXd u_min_;

        State state_;
        float d;
        // mode
};