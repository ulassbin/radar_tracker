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
    /**
     * @brief Visualizes gates of measurements.
     * @param x is the X position of track
     * @param y is the Y position of track
     * @param sk_inv is the inverse of tracks covariance after prediction step. This value allows the gate
     * to weigh the dimensions and accept - reject matches based on uncertainty of filter.
     * @param tol is the acceptenca tolerance value of gate
     * @param id is the id of the track.
     * @details These gates are used to select, which measurements are accepted to correspondence set
     * If a position - velocity combination falls inside these n dimensional elipses, it will be 
     * fed to the filters as measurement.
     */
    void visualizeTwo2DGates(double x, double y, Eigen::MatrixXd sk_inv, double tol, int id);
  private:
    ros::Publisher pub_;
};
};
#endif