#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <radar_estimator/common.h>
#include <radar_estimator/visualizer.h>
#include <radar_estimator/kalman_filter.h>
#include <radar_estimator/Filter.h>
#include <radar_estimator/EstimatorDebug.h>

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
    * @brief Checks if canditates are more probable than no match
    * @param candidates are the set of measurements that have fallen in the gate of track.
    * @param normalizer is the returned normalizer.It iss the normalization value for 
    * the weights b0, b1, ... bn where n is the number of candidates.
    * @return Returns if b1 to bn is larger than b0 (possibility of no match.)
    * @details currently, I tried to discard the denominator CV(Poisson distributed clutter)
    * part by taking it as a constant value, and using a normalizer to make sure 
    * b0 + b1 + ... bn = 1.
    * This approach kind of didn't work, still working on it.(Usuall b0 > b1 + ... bn)
    */
    bool checkMatchValidity(std::vector<std::pair<int,double>> candidates, double& normalizer);
    /**
    * @brief Gets nearest neighbour to measurement
    * @param meas is the measurement to be checked
    * @return int is the index of closest tracker
    * @details this was previously used in Nearest Neighbour approach
    * kept here just in case.
    */
    int getClosestTracker(const measurement meas);
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
     * @details Calculates gates of all trackers, finds the measurements
     * that fall into each track's gate. Their innovation and, measurement indexes are saved
     * to a map. Map is used here, since if there is no element is pushed, map still returns a blank vector
     * pair.
     * @param PD is the probability of detection
     * @return corresp_map is the set of corresponding measurements to each trackers
     * key:tracker_id, value: 1st: measurement index, 2nd: gaussian value obtained with innovation.
     */
    std::map<int, std::vector<std::pair<int, double>>> getMatchesInGate(double PD);

    /**
     * @details Broadcasts the debug msg formed by filter states.
     */
    void broadcastDebugMsg();

    /**
     * @details Visualizes all tracked filter states.
     */
    void visAllFilters();
    ros::Subscriber sub_msgs_;
    ros::Publisher pub_debug_;
    ros::Timer timer_;
    ros::NodeHandle private_nh_;

    bool got_msg_ = false;

    visualizer::visualizer uviz_;
    std::vector<kalman_filter::kalmanFilter> trackers_;
    std::vector<measurement> measurements_;
    radar_estimator::EstimatorDebug debug_;

};
};

#endif