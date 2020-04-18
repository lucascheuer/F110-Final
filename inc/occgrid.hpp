#include <ros/ros.h>
#include <Eigen/Geometry>

class OccGrid
{
    public:
        OccGrid(int size);
        virtual ~OccGrid();
    private:
        int size_;
        Eigen::MatrixXf grid_;
        std::pair<int, int> GridPoint(float x, float y);
        // mode
};