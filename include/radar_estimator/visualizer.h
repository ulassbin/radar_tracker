#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <radar_estimator/common.h>

typedef common::measurement measurement;
namespace visualizer
{
class visualizer
{
  public:
    visualizer();
    void visualizeArrays(std::vector<measurement> measurements);
  private:
    ros::Publisher pub_;

};
};
#endif