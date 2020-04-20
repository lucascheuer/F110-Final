#ifndef VIZ_H
#define VIZ_H
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

class Visualizer
{
    public:
        static std::vector<geometry_msgs::Point> GenerateVizPoints(std::vector<std::pair<float,float>> &points);
        static std::vector<std_msgs::ColorRGBA> GenerateVizColors(std::vector<std::pair<float,float>> &points, float r = 1, float g = 1, float b = 1);
        static visualization_msgs::Marker GenerateList(std::vector<std::pair<float,float>> &points, std::vector<std_msgs::ColorRGBA> &marker_colors, int type = visualization_msgs::Marker::SPHERE_LIST);
        static visualization_msgs::Marker GenerateSphereList(std::vector<std::pair<float,float>> &points, float r = 1, float g = 1, float b = 1);
        static visualization_msgs::Marker GenerateSphereList(std::vector<std::pair<float,float>> &points, std::vector<std_msgs::ColorRGBA> &marker_colors);


};

#endif