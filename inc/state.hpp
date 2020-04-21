#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>
#include <Eigen/Geometry>

class State
{
    public:
        State();
        virtual ~State();

        // setters
        void set_x(double x);
        void set_y(double y);
        void set_ori(double z);

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