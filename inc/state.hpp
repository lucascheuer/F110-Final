#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>
#include <Eigen/Geometry>

class State
{
    public:
        State();
        State(double x, double y, double ori);
        virtual ~State();

        Eigen::VectorXd ToVector();

        // setters
        void SetX(double x);
        void SetY(double y);
        void SetOri(double z);
        std::pair<float,float> getPair();
        // getters
        double x();
        double y();
        double ori();
        int size();
    private:
        double x_;
        double y_;
        double ori_;
        int size_;
        //... fill with state variables
        
};

#endif