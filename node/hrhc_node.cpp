#include "hrhc.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hrhc");
    ros::NodeHandle nh;
    HRHC hrhc(nh);
    ros::spin();
    return 0;
}