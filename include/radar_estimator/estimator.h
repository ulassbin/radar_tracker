#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <radar_estimator/common.h>
#include <radar_estimator/visualizer.h>
#include <radar_estimator/kalman_filter.h>

namespace estimator
{
/**
 * @brief Target Estimating Class
 * @details Estimates the objects, by subscribing to topics,
 * forming filters, handling prediction-measurement steps
 * and calling visualization class when necessary
 */
class estimator
{
  public:
    estimator();
   ~estimator();
  private:
    /**
    * @brief Callback to handle messages
    * @details the data is processed here as well (Which is bad, data should be processed in another thread)
    * blocking this callback is not good. Return back when you have time
    */
    void messageCallback(const std_msgs::Float32MultiArray msg);
    /**
    * @brief Main loop callback
    */
    void timerCallback(const ros::TimerEvent& event);
    /**
    * @brief Gets nearest neighbour to measurement
    * @param meas is the measurement to be checked
    * @return int is the index of closest tracker
    */
    int getClosestTracker(const measurement meas);

    ros::Subscriber sub_msgs_;
    ros::Timer timer_;
    ros::NodeHandle private_nh_;

    bool got_msg_ = false;

    visualizer::visualizer uviz_;
    std::vector<kalman_filter::kalmanFilter> trackers_;
    std::vector<measurement> measurements_;
};
};

#endif