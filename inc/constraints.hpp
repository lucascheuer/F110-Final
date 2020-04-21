#include <ros/ros.h>
#include <Eigen/Geometry>

class Constraints
{
    public:
        Constraints();
        virtual ~Constraints();
        Eigen::MatrixXf getXMax();
        Eigen::MatrixXf getUMax();
        Eigen::MatrixXf getXMin();
        Eigen::MatrixXf getUMin();

        void setXMax(Eigen::MatrixXf xmax);
        void setUMax(Eigen::MatrixXf umax);
        void setXMin(Eigen::MatrixXf xmin);
        void setUMin(Eigen::MatrixXf umin);

        void setXlims(float x,float y); // sets xmax, xmin at +-d
        // void setUlims(float x,float y); 

    private:
        Eigen::MatrixXf XMax_;
        Eigen::MatrixXf UMax_;
        Eigen::MatrixXf XMin_;
        Eigen::MatrixXf UMin_;
        float d;
        // mode
};