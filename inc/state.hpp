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

        // getters
        double x();
        double y();
        
    private:
        double x_;
        double y_;
        //... fill with state variables
};