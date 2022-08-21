#include <radar_estimator/kalman_filter.h>

namespace kalman_filter
{
  kalmanFilter::kalmanFilter()
  {
  	// max vol in w-h-m is around 3m so +-1.5m initial guess is good
  	ROS_INFO("Constructed blank filter");
  	cov_ = Eigen::MatrixXd(7, 7);
  	cov_ << 100,0,0,0,0,0,0, // Cov0 init 10m mistake initially
  	        0,100,0,0,0,0,0, // Velocity? lets say +-1.25 (From min-max in data)
  	        0,0,0,1.6,0,0,0,
  	        0,0,0,0,1.6,0,0,
  	        0,0,0,0,2.25,0,0,
  	        0,0,0,0,0,2.25,0,
  	        0,0,0,0,0,0,2.25;
    Qt_ = Eigen::MatrixXd(7,7);
    Qt_ << 4,0, 0,0, 0,0,0,
          0,4, 0,0, 0,0,0,
          0,0, 0.25,0, 0,0,0,
          0,0, 0,0.25, 0,0,0,
          0,0, 0,0, 0.2,0,0,
          0,0, 0,0, 0,0.2,0,
          0,0, 0,0, 0,0,0.2;

  	mean_ = Eigen::VectorXd(7);
  	mean_ << 5, 5, 0, 0, 1.5, 1.5, 1.5;
  	cov_p_ = cov_;
  	mean_p_ = mean_;
  }

  kalmanFilter::~kalmanFilter()
  {
  	ROS_INFO("Goodbye");
  }

  void kalmanFilter::assign(measurement meas)
  {
  	mean_ = common::measurementToEigen(meas);
  }
  
  void kalmanFilter::iterate(measurement meas)
  {
  	prediction();
  	ROS_INFO("Done prediction");
  	measurementUpdate(common::measurementToEigen(meas));
  	ROS_INFO("Update cycle succesful");
  }

  void kalmanFilter::prediction()
  {
    ROS_INFO("Prediction");	
  	double delt = 1 / 14.0; // This is hardcoded, its basically message frequency or stamp diff
  	Eigen::MatrixXd At(7,7);
  	//    x,y,xd,yd,volx,voly,volz
  	At << 1,0,delt,0,0,0,0,
  	      0,1,0,delt,0,0,0,
  	      0,0,1,0   ,0,0,0,
  	      0,0,0,1   ,0,0,0,
  	      0,0,0,0   ,1,0,0,
  	      0,0,0,0   ,0,1,0,
  	      0,0,0,0   ,0,0,1;
  	//Bt = 0;
  	mean_p_ = At*mean_;
  	ROS_INFO("Formed mean_p");
  	Eigen::MatrixXd Rt(7,7);
  	// Process noise lets say 1m for position + 0.07*noise_in_velocity =~ 2?
  	double acc = 0.1; // Process noise could be acceleration term
  	Rt << pow(delt,4)*acc,0, pow(delt,3)*acc/2,0, 0,0,0,
  	      0,pow(delt,4)*acc, 0,pow(delt,3)*acc/2, 0,0,0,
  	      0,0, delt*delt*acc,0, 0,0,0,
  	      0,0, 0,delt*delt*acc, 0,0,0,
  	      0,0, 0,0, 0.1,0,0,
  	      0,0, 0,0, 0,0.1,0,
  	      0,0, 0,0, 0,0,0.1;
  	cov_p_ = At*cov_*At.transpose() + Rt;
  }

  void kalmanFilter::pdaUpdate(std::vector<std::pair<int,double>> match_vect, std::vector<measurement> measurements, double PD, double norm)
  {
    std::vector<Eigen::VectorXd> innovations;
    std::vector<double> weights;
    Eigen::VectorXd inv(7);
    double b0 = (1-PD)*norm;
    Eigen::MatrixXd Kgain(7,7);
    Eigen::MatrixXd Ct(7,7);
    Ct = Eigen::MatrixXd::Identity(7,7);
    Eigen::MatrixXd Pk(7,7);
    Pk = Eigen::MatrixXd::Zero(7,7);
    Eigen::MatrixXd Sk(7,7);
    Eigen::VectorXd weighted_inv(7);
    weighted_inv = Eigen::VectorXd::Zero(7);

    Sk = Ct*cov_p_*Ct.transpose()+Qt_;
    Kgain = (cov_p_*Ct.transpose())*Sk.inverse();
    for(int id = 0; id < match_vect.size(); id++)
    {
      inv = (common::measurementToEigen(measurements[match_vect[id].first]) - Ct*mean_p_);
      innovations.push_back(inv);
      weights.push_back(match_vect[id].second*norm);
      weighted_inv += (weights[id]) * inv;
    }
    double add_cov = 0;
    for(int i = 0; i < innovations.size(); i++)
    {
      Eigen::VectorXd temp(7);
      temp = innovations[i];
      double val1 = temp.transpose() * temp;
      double val2 = weighted_inv.transpose() * weighted_inv;
      add_cov += (weights[i] * val1 - val2);
    }
    Pk = Kgain * add_cov *Kgain.transpose();
    // Pk|k = cov_
    cov_ = cov_p_ + (1-b0)*Kgain*Sk*Kgain.transpose() + Pk;
    mean_ += Kgain*weighted_inv;
  }


  void kalmanFilter::measurementUpdate(Eigen::MatrixXd measurement)
  {
  	Eigen::MatrixXd Ct(7,7);
  	Ct = Eigen::MatrixXd::Identity(7,7);
  	Eigen::MatrixXd Kgain(7,7);
  	//Qt = Eigen::MatrixXd::Identity(7,7); // We measure all..
  	// Estimated standard devs for this task
  	// Position +-2m ( Check catalog for this)
  	// velocity 0.5m/s lets say
  	// Volume similary lets take it as +-2m
  	// Values down belwo are squared values of stddev -> variance.
  	Kgain = cov_p_*Ct.transpose()*((Ct*cov_p_*Ct.transpose()+Qt_).inverse());

  	mean_ = mean_p_ + Kgain*(measurement - Ct*mean_p_);
  	cov_ = (Eigen::MatrixXd::Identity(7,7) - Kgain*Ct)*cov_p_;
  }

};