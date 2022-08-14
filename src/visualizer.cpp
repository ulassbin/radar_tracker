#include <radar_estimator/visualizer.h>

namespace visualizer
{
  visualizer::visualizer()
  {
  	ros::NodeHandle nh;
    pub_ = nh.advertise<visualization_msgs::Marker>("/sensor_visualization", 10, false);
  }

  void visualizer::visualizeArrays(std::vector<measurement> measurements)
  {
  	visualization_msgs::Marker marker;
  	int count = 0;
  	marker.pose.orientation.x = 0;
  	marker.pose.orientation.y = 0;
  	marker.pose.orientation.z = 0;
  	marker.pose.orientation.w = 1;
  	marker.pose.position.z = 0;
  	marker.color.r = 0;
  	marker.color.g = 1.0;
  	marker.color.b = 0.0;
  	marker.color.a = 1.0;
  	marker.header.frame_id = "sensor";
  	marker.header.stamp = ros::Time::now();
    marker.action = 0;
    marker.type = 1;

  	for(auto measurement : measurements)
  	{
  	  marker.ns = std::to_string(count);
  	  marker.id = count;
  	  marker.scale.x = measurement.d_;
  	  marker.scale.y = measurement.w_;
  	  marker.scale.z = measurement.h_;
  	  marker.pose.position.x = measurement.x_;
  	  marker.pose.position.y = measurement.y_;

  	  ROS_INFO("Passing Trough measurement %d",measurement.id_);
  	  count++; // Using local ids;
  	  pub_.publish(marker);
  	}
  }
};