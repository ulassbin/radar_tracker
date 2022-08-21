#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

namespace common
{
  struct measurement
  {
  	int type_;
  	int no_;
  	int id_;
  	double x_;
  	double y_;
  	double x_vel_;
  	double y_vel_;
  	double w_;
  	double h_;
  	double d_;
  	void parse(std::vector<double> d)
  	{
  	  type_ = d[0];
  	  no_ = d[1];
  	  id_ = d[2];
  	  x_ = d[3];
  	  y_ = d[4];
  	  x_vel_ = d[5];
  	  y_vel_ = d[6];
  	  w_ = d[7];
  	  h_ = d[8];
  	  d_ = d[9];
  	}
  };

  Eigen::VectorXd measurementToEigen(const measurement meas)
  {
  	Eigen::VectorXd m_eig(7);
  	m_eig << meas.x_, meas.y_,
             meas.x_vel_, meas.y_vel_,
             meas.w_, meas.h_, meas.d_;
    return m_eig;
  }

  Eigen::VectorXd toEigen(const measurement meas)
  {
    Eigen::VectorXd vect(7);
    vect << meas.x_, meas.y_, meas.x_vel_, meas.y_vel_,
            meas.w_, meas.h_, meas.d_;
    return vect;
  }

  std::vector<measurement> parseMeasurements(const std_msgs::Float32MultiArray msg)
  {
  	std::vector<measurement> measurements;
  	measurement temp;
  	std::vector<double> temp_d;
  	if(msg.data.size() == 140)
  	{
  	  for(int obj_id = 0; obj_id < 10; obj_id++)
  	  {
  	  	temp_d.clear();
  	  	for(int feat_id = 0; feat_id < 14; feat_id++)
  	  	{
  	  	  temp_d.push_back(msg.data[obj_id + feat_id*10]);
  	  	}
  	  	temp.parse(temp_d);
  	  	measurements.push_back(temp);
  	  }
  	}
  	else
  	{
  	  ROS_WARN("Message size mismatch %d, skipping", int(msg.data.size()));
  	}
  	return measurements;
  };

  double getGaussianValue(Eigen::MatrixXd cov, Eigen::VectorXd diff)
  {
   //p(x) = det(2*pi*Σ)^(-1/2)*exp(-1/2*(x-µ)^T*Σ^-1*(x-µ))
    // diff = (x-µ)
    double denum = sqrt((2*M_PI*cov).determinant());
    double num = exp(-1/2.0 * diff.transpose() * cov.inverse() * diff);
    return num/denum;
  }
};

#endif