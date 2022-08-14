#include <radar_estimator/estimator.h>
#include <ros/ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "radar_estimator");
  ros::NodeHandle nh;
  estimator::estimator estim;
  ros::spin();
}