#ifndef KALMAN_H
#define KALMAN_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <radar_estimator/common.h>


namespace kalman_filter
{
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
  typedef common::measurement measurement;
  //System model 
  //Xt = A*xt-1 + Bt*Ut + epsilon
  //Sensor model
  // Yk = HkXk + Vk
  // x = { xpos,ypos, xdot,ydot}
  public:
  kalmanFilter();
  ~kalmanFilter();
  /**
  * @brief Iterates the filter with measurement 
  * @param meas is the measurement input to filter.
  * @details Calls the prediction step, and then measurement Update Method.
  */
  void iterate(measurement meas);
  /**
   * @brief Forms filters initial state
   * @param meas is the first percieved input to start the filter
   */
  void assign(measurement meas);
  Eigen::MatrixXd cov_;//(7,7);
  Eigen::VectorXd mean_;//(7);
  bool first_ = true;
  private:
  /**
  * @brief Makes a prediction with current states.
  * @details Changes mean and cov values of filter by 
  * just using previous values.
  */
  void prediction();
  /**
   * @brief Standard KF measurement update
   * @param measurement is the measurement raw values in Eigen Vect format
   */
  void measurementUpdate(Eigen::MatrixXd measurement);
  Eigen::MatrixXd cov_p_;//(7,7);
  Eigen::VectorXd mean_p_;//(7);
  std::string name_;
  std::string state_;
  std::vector<double> process_noise_;
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