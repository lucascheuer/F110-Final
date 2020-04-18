#include <ros/ros.h>
#include <Eigen/Geometry>

class Model
{
    public:
        Model();
        virtual ~Model();
    private:
        Eigen::MatrixXf a_;
        Eigen::MatrixXf b_;
        Eigen::MatrixXf c_;
        double time_step_;
        // mode
};