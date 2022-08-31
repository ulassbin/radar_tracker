#ifndef KALMAN_H
#define KALMAN_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <radar_estimator/common.h>
#include <radar_estimator/Filter.h>

namespace kalman_filter
{
typedef common::measurement measurement;

/**
 * @brief This is a standard kalman filter implementation
 * @details This class holds necessary methods to keep track of objects
 * that are defined as tracks. 
 * Mostly eigen library is referred to carry out matrix operations. 
 * System model is assumed to be linear and time-invariant. 
 * This filter works by continously forming a prediction at each new timestep,
 * and updating that prediction whenever a new information arrives from a sensor.
 * How sensor information is relayed to this filter is not a concern of this class.
 */

class kalmanFilter
{
  //System model 
  //Xt = A*xt-1 + Bt*Ut + epsilon
  //Sensor model
  // Yk = HkXk + Vk
  // x = { xpos,ypos, xdot,ydot, ...}
  public:
  /**
   * @brief Class constructor with id.
   */
  kalmanFilter(int i, std::string state);
  /**
   * @brief Class constructor with id.
   */
  kalmanFilter(int i) : kalmanFilter(i, "initial") {};
  /**
   * @brief Destructor definition
   * @details This could be useful for removing unnecessary tracks later on.
   */
  ~kalmanFilter();
  /**
  * @brief Iterates the filter with measurement 
  * @param meas is the measurement input to filter.
  * @details Calls the prediction step, and then measurement Update Method.
  */
  void iterate(measurement meas);
  /**
  * @brief Makes a prediction with current states.
  * @details Changes mean and cov values of filter by 
  * just using previous values.
  */
  void prediction(); 
  /**
  * @brief Updates the filter with measurement set.
  * @param match_vect is the set of corresponding measurements to this filter.
  * @param measurements is the measurement vector, which holds the raw values
  * @param PD is the probability of detection (This is used in updating step)
  * @param norm is the normalization factor for weights b0,b1,b2 ... which are 
  * the weights that average the correspondences.
  * @details For this filter, match_vect holds the measurement index and the value obtained
  * from the gaussian with the measurements innovation. This innovation is carried to here
  * not to recompute values from gaussian. It was previously computed at gating method.
  * This function, will update the state of the filter with the weights and the content
  * of the set of matches. It is basically a kalman filter update step with more than
  * one measurements, but they are weighted. 
  */
  void pdaUpdate(std::vector<std::pair<int,double>> match_vect, std::vector<measurement> measurements, double PD, double norm);
  /**
   * @brief Forms filters initial state
   * @param meas is the first percieved input to start the filter
   */
  void assign(measurement meas);
  Eigen::MatrixXd cov_;//(7,7);
  Eigen::VectorXd mean_;//(7);
  Eigen::MatrixXd cov_p_;//(7,7);
  Eigen::VectorXd mean_p_;//(7);
  bool first_ = true;
  radar_estimator::Filter msg_;
  std::string state_;

  private:
  /**
   * @brief Standard KF measurement update
   * @param measurement is the measurement raw values in Eigen Vect format
   */
  void measurementUpdate(Eigen::VectorXd measurement);
  
  /**
   * @brief Updates msg to currrent state
   */
  void updateMsg();
  int id_;
  std::vector<double> process_noise_;
  Eigen::MatrixXd Qt_;
};
// p(x) is a gaussian
// p(x) = det(2*pi*Σ)^(-1/2)*exp(-1/2*(x-µ)^T*Σ^-1*(x-µ))
// p(x) basicallly composed of mean µ and covariance Σ.
// Posterior(Prediction), given Ut and Xt-1
// p(xt|ut,xt-1) = det(2*pi*Rt)^-1/2 *
// exp(-1/2(Xt-AtXt-1-BtUt)^T*Rt^-1*(-1/2(Xt-AtXt-1-BtUt)^)
// Zt = Ct*Xt + εt1
// P(Zt | xt) = det(2*pi*Qt)^(-1/2)*
// exp(-1/2(Zt-Ct*Xt)^T*Qt^-1*(Zt-Ct*Xt))
};
#endif