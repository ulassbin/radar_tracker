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
/**
* @brief Visualization Handler.
* @details This class holds necessary methods to visualize raw sensor data,
* filter states, covariances, gates for PDA, JPDA ... etc.
* Currently only visualization_marker is supported. 
*/    
class visualizer
{
  public:
    visualizer();
    /**
    * @brief Visualizes raw measurements.
    * @param measurements are the raw measurements directly parsed from ros message input.
    */ 
    void visualizeArrays(std::vector<measurement> measurements);
    /**
    * @brief Visualizes a tracked object.
    * @param mean is the raw values of filter. Positions, velocities and dimensions are all used here.
    * @param cov is the covariance of filter.
    * @param ns is just a namespace parameter to select-deselect during rviz visualization.
    */ 
    void visFilterStates(Eigen::VectorXd mean, Eigen::MatrixXd cov, std::string ns);
  private:
    ros::Publisher pub_;
};
};
#endif