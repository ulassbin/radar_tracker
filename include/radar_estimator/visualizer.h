#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <radar_estimator/common.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef common::measurement measurement;
namespace visualizer
{
class visualizer
{
  public:
    visualizer();
    void visualizeArrays(std::vector<measurement> measurements);
    void visFilterStates(Eigen::VectorXd mean, Eigen::MatrixXd cov, std::string ns);
  private:
    ros::Publisher pub_;
};
};
#endif