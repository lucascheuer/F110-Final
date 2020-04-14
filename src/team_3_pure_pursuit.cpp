#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <cmath>
#include <fstream>
#include <tf2/transform_datatypes.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <experimental/filesystem>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>

// TODO: include ROS msg type headers and libraries you need
using namespace std;
namespace fs = experimental::filesystem;
class PurePursuit {
private:
    ros::NodeHandle n;
    ros::Subscriber poseSubscriber;
    ros::Subscriber scanSubscriber;
    ros::Publisher drivePublisher;
    ros::Publisher vis_pub_mult;
    ros::Publisher vis_pub;
    vector<pair<float,float>> waypoints;
    visualization_msgs::MarkerArray waypoint_markers;
    bool printed = false;
    const float LOOKAHEAD_REF = 1.2;
    const float BIG_FLOAT = 9999999;
    const float MAX_VELOCITY = 4.5;
    const float MAX_ANGLE = .41;
    float leftSideDistance;
    bool successfulRead;
public:
    PurePursuit() : successfulRead(false) {
        n = ros::NodeHandle();
        // sim
        poseSubscriber = n.subscribe("/odom", 1, &PurePursuit::pose_callback, this);
//         scanSubscriber = n.subscribe("scan", 1, &PurePursuit::scan_callback, this);

        // real
        //poseSubscriber = n.subscribe("pf/pose/odom", 1, &PurePursuit::pose_callback, this);
        drivePublisher = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
        getWaypoints();
        vis_pub_mult = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10 );
        vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10 );
        gen_markers();
    }
    
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) 
    {
        std::vector<float> rangeData = msg->ranges;
        float increment = msg->angle_increment;
        float angle_min = msg->angle_min;
        float angle_max = msg->angle_max;
        float minTTC = __INT_MAX__;
        float scanAngle = angle_min;
        leftSideDistance = rangeData[810];
//         cout << leftSideDistance << endl;
    }

    void getWaypoints()
    {
        string path = ros::package::getPath("team_3_pure_pursuit")+"/fooxx.csv";
        // string path = string(fs::current_path()) + "/src/f110_ros/team_3_pure_pursuit/src/foo.csv";
        cout << path << endl;
        ifstream input(path);
        string poseX, poseY,theta,speed;
        if (input.is_open()) {
            while(getline(input,poseX,',')) {
                getline(input,poseY,',');
                getline(input,theta,',');
                getline(input,speed);
                waypoints.push_back(pair<float,float>(stof(poseX),stof(poseY)));
            }
        } else {
            cout << "Please run this from the root catkin_ws directory" << endl;
            exit(0);
        }
    }

    void gen_markers()
    {
        ROS_INFO("%zu", waypoints.size());
        for (int ii = 0; ii < waypoints.size(); ii += 10)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = ii;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = waypoints[ii].first;
            marker.pose.position.y = waypoints[ii].second;
            marker.pose.position.z = 0.1;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            waypoint_markers.markers.push_back(marker);
        }
        
    }

    vector<float> getDistances(vector<pair<float, float>> waypoints, geometry_msgs::TransformStamped transform_msg)
    {
        vector<float> distances;
        geometry_msgs::Vector3 waypoint_in;
        geometry_msgs::Vector3 waypoint_out;
        for (auto itr = waypoints.begin(); itr != waypoints.end(); itr++) {
            pair<float, float> point = *itr;
            waypoint_in.x = point.first;
            waypoint_in.y = point.second;
            waypoint_in.z = 0;
            tf2::doTransform(waypoint_in, waypoint_out, transform_msg);
            // this rotates the point into the car frame
            // then we need to account for car's pose
            waypoint_out.x += transform_msg.transform.translation.x;
            waypoint_out.y += transform_msg.transform.translation.y;
            //cout << "in" << waypoint_in << "out" << waypoint_out << endl;
            if (waypoint_out.x > 0.25) { // TODO: check this behaviour 
                distances.push_back(sqrt(waypoint_out.x*waypoint_out.x+waypoint_out.y*waypoint_out.y));
            } else {
                distances.push_back(BIG_FLOAT);
            }
        }
        return distances;
    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
        
        // TODO: find the current waypoint to track using methods mentioned in lecture
        auto pose_real_msg = pose_msg->pose;
        vis_pub_mult.publish( waypoint_markers );
        float poseX = pose_real_msg.pose.position.x;
        float poseY = pose_real_msg.pose.position.y;
//         cout << poseX << "," << poseY << ","<<leftSideDistance<<endl;
        tf2::Transform car_to_word;
        
        tf2::fromMsg(pose_msg->pose.pose, car_to_word);
        
        tf2::Transform world_to_car = car_to_word.inverse();
        
        geometry_msgs::Transform world_to_car_msg = tf2::toMsg(world_to_car);
        geometry_msgs::TransformStamped world_to_car_msg_stamped;
        world_to_car_msg_stamped.transform = world_to_car_msg;

        vector<float> distances = getDistances(waypoints, world_to_car_msg_stamped);
        float minDistance = BIG_FLOAT;
        float argminDist = -1;
        for (int i = 0; i < distances.size(); i++) {
            float currentDistance = distances[i] - LOOKAHEAD_REF;
            if (currentDistance >= 0 && currentDistance < minDistance)
            {
                argminDist = i;
                minDistance = currentDistance;
            }
        }
        pair<float,float> targetWaypoint = waypoints[argminDist];
        
        float realLookahead = distances[argminDist];
        // TODO: transform goal point to vehicle frame of reference
        publish_marker(targetWaypoint.first, targetWaypoint.second);
        geometry_msgs::Vector3 targetWaypointVector;
        geometry_msgs::Vector3 transformedWaypointVector;
        targetWaypointVector.x = targetWaypoint.first;
        targetWaypointVector.y = targetWaypoint.second;
        targetWaypointVector.z = 0;
        tf2::doTransform(targetWaypointVector,transformedWaypointVector,world_to_car_msg_stamped);
        transformedWaypointVector.x += world_to_car_msg_stamped.transform.translation.x;
        transformedWaypointVector.y += world_to_car_msg_stamped.transform.translation.y;
        // TODO: calculate curvature/steering angle
        float y = transformedWaypointVector.y;
        float curvature = (realLookahead*realLookahead)/(2*y);
        float angle = 1/curvature;
        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = angle;
        angle = (angle>MAX_ANGLE) ? MAX_ANGLE : angle;
        angle = (angle<-MAX_ANGLE) ? -MAX_ANGLE : angle;
        drive_msg.drive.speed = MAX_VELOCITY*(1-0.0*angle/MAX_ANGLE);
        drivePublisher.publish(drive_msg);
    }

    void publish_marker(float x, float y)
    {
//         cout << x << "," << y << endl;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "current";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        vis_pub.publish(marker);
    }
};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}
