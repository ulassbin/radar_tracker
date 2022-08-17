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
class estimator
{
  public:
    estimator();
   ~estimator();
  private:
    void messageCallback(const std_msgs::Float32MultiArray msg);
    void timerCallback(const ros::TimerEvent& event);
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