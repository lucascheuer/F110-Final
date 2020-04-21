#include <ros/ros.h>
#include <Eigen/Geometry>

class Constraints
{
    public:
        Constraints();
        virtual ~Constraints();
        Eigen::MatrixXf x_max();
        Eigen::MatrixXf u_max();
        Eigen::MatrixXf x_min();
        Eigen::MatrixXf u_min();

        void set_x_max(Eigen::MatrixXf xmax);
        void set_u_max(Eigen::MatrixXf umax);
        void set_x_min(Eigen::MatrixXf xmin);
        void set_u_min(Eigen::MatrixXf umin);

        void SetXLims(float x,float y); // sets xmax, xmin at +-d
        // void setUlims(float x,float y); 

    private:
        Eigen::MatrixXf x_max_;
        Eigen::MatrixXf u_max_;
        Eigen::MatrixXf x_min_;
        Eigen::MatrixXf u_min_;
        float d;
        // mode
};