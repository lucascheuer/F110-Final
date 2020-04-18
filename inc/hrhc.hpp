#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
class HRHC
{
    public:
        HRHC(ros::NodeHandle &nh);
        virtual ~HRHC();
    private:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};