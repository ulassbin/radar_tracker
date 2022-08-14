#include <radar_estimator/estimator.h>

namespace estimator
{
  estimator::estimator()
  {
  	ros::NodeHandle nh;
  	got_msg_ = false;
  	sub_msgs_ =  nh.subscribe("/lrrObjects", 10, &estimator::messageCallback, this);
    //timer_ = private_nh_.createTimer(ros::Duration(0.1), std::bind(&estimator::timerCallback, this, _1));
  }

  estimator::~estimator()
  {
  	ROS_INFO("Goodbye");
  }

  void estimator::messageCallback(const std_msgs::Float32MultiArray msg)
  {
  	measurements_ = common::parseMeasurements(msg); // Store to array could be better if processing freq is low.
  	got_msg_ = true;
  }

  void estimator::timerCallback(const ros::TimerEvent& event)
  {
  	//Process stuff here.
  	if(got_msg_)
  	{
  	  ROS_INFO("Processing");
  	  uviz_.visualizeArrays(measurements_);
  	  got_msg_ = false;
  	}
  	else
  	{
  	  ROS_DEBUG("Skipping no msg");
  	}
  }
};