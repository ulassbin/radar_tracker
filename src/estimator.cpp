#include <radar_estimator/estimator.h>

namespace estimator
{
  estimator::estimator()
  {
  	ros::NodeHandle nh;
  	got_msg_ = false;
  	sub_msgs_ =  nh.subscribe("/lrrObjects", 10, &estimator::messageCallback, this);
    //timer_ = private_nh_.createTimer(ros::Duration(0.1), std::bind(&estimator::timerCallback, this, _1));
    for(int i = 0; i<10; i++)
      trackers_.push_back(kalman_filter::kalmanFilter());
  }

  estimator::~estimator()
  {
  	ROS_INFO("Goodbye");
  }

  void estimator::messageCallback(const std_msgs::Float32MultiArray msg)
  {
  	measurements_ = common::parseMeasurements(msg); // Store to array could be better if processing freq is low.
    uviz_.visualizeArrays(measurements_);
    for(int i = 0; i < measurements_.size(); i++)
    {
      ROS_INFO("Iterating %d", i);
      if(trackers_[i].first_)
      {
      	trackers_[i].assign(measurements_[i]);
      	trackers_[i].first_ = false;
      }
      else
      	trackers_[i].iterate(measurements_[i]); // These correspondences are going to be calculated via JIPDA
      ROS_INFO("Filtered %d", i);
      uviz_.visFilterStates(trackers_[i].mean_, trackers_[i].cov_, std::to_string(i));
      ROS_INFO("Visualized %d", i);
    }
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