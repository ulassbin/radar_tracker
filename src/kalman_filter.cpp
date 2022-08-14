#include <radar_estimator/kalman_filter.h>

namespace kalman_filter
{
  kalmanFilter::kalmanFilter()
  {
  	ROS_INFO("Constructed blank filter");
  	cov_ = Eigen::MatrixXd(7,7);
  	mean_ = Eigen::VectorXd(7);
  	cov_p_ = Eigen::MatrixXd(7,7);
  	mean_p_ = Eigen::VectorXd(7);
  }

  kalmanFilter::~kalmanFilter()
  {
  	ROS_INFO("Goodbye");
  }

  void kalmanFilter::prediction()
  {
    ROS_INFO("Prediction");	
  	double delt = 0.05 / 2.0;
  	Eigen::MatrixXd At(7,7);
  	//    x,y,xd,yd,volx,voly,volz
  	At << 1,0,delt,0,0,0,0,
  	      0,1,0,delt,0,0,0,
  	      0,0,1,0,0,0,0,
  	      0,0,0,1,0,0,0,
  	      0,0,0,0,1,0,0,
  	      0,0,0,0,0,1,0,
  	      0,0,0,0,0,0,1;
  	//Bt = 0;
  	mean_p_ = At*mean_;
  	Eigen::VectorXd Rt(7);
  	Rt << 0.05,0.05,0.1,0.1, // Posx,Posy,Vx,Vy
  	      0.2,0.2,0.2; //Volx,Voly,Volth
  	cov_p_ = At*cov_*At.transpose() + Rt.transpose();
  }

  void kalmanFilter::measurementUpdate(Eigen::MatrixXd measurement)
  {
  	Eigen::MatrixXd Ct(7,7);
  	Ct = Eigen::MatrixXd::Identity(7,7);
  	Eigen::MatrixXd Kgain(7,7);
  	Eigen::MatrixXd Qt(7,7);
  	Qt = Eigen::MatrixXd::Identity(7,7);
  	Qt << 0.25,0,0,0,0,0,0,
  	      0,0.25,0,0,0,0,0,
  	      0,0,0,0.05,0,0,0,
  	      0,0,0,0,0.05,0,0,
  	      0,0,0,0,0,0.5,0,0,
  	      0,0,0,0,0,0,0.5,0,
  	      0,0,0,0,0,0,0,0.5;
  	Kgain = cov_p_*Ct*(Ct*cov_p_*Ct.transpose()+Qt).inverse();

  	mean_ = mean_p_ + Kgain*(measurement - Ct*mean_p_);
  	cov_ = (Eigen::MatrixXd::Identity(7,7) - Kgain*Ct)*cov_p_;
  }

};