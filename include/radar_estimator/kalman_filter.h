#ifndef KALMAN_H
#define KALMAN_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <radar_estimator/common.h>


namespace kalman_filter
{
class kalmanFilter
{
  typedef common::measurement measurement;
  //System model 
  //Xt = A*xt-1 + Bt*Ut + epsilon
  //Sensor model
  // Yk = HkXk + Vk
  // x = { xpos,ypos, xdot,ydot}
  public: 
  kalmanFilter(int i);
  ~kalmanFilter();
  void iterate(measurement meas);
  void prediction(); // This is in public to just get values before gating.
  void pdaUpdate(std::vector<std::pair<int,double>> match_vect, std::vector<measurement> measurements, double PD, double norm);
  void assign(measurement meas);
  Eigen::MatrixXd cov_;//(7,7);
  Eigen::VectorXd mean_;//(7);
  Eigen::MatrixXd cov_p_;//(7,7);
  Eigen::VectorXd mean_p_;//(7);
  bool first_ = true;
  private:
  void measurementUpdate(Eigen::MatrixXd measurement);
  int id_;
  std::string state_;
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