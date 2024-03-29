#include <radar_estimator/visualizer.h>

namespace visualizer
{
  visualizer::visualizer()
  {
  	ros::NodeHandle nh;
    pub_ = nh.advertise<visualization_msgs::Marker>("/sensor_visualization", 10, false);
  }

  void visualizer::visualizeGateVectors(double trackx, double tracky, double measx, double measy, double track_id)
  {
  	visualization_msgs::Marker marker;
    marker.ns = "gatemap";
  	int count = 0;
  	marker.pose.orientation.x = 0;
  	marker.pose.orientation.y = 0;
  	marker.pose.orientation.z = 0;
  	marker.pose.orientation.w = 1;
  	marker.pose.position.z = 0;
  	marker.color.r = 0;
  	marker.color.g = 1.0;
  	marker.color.b = 1.0;
  	marker.color.a = 1.0;
  	marker.header.frame_id = "sensor";
  	marker.header.stamp = ros::Time::now();
    marker.type = 4; // Line strip
    
    //marker.action = 2; // Clear here
    //pub_.publish(marker);
    marker.pose.orientation.w = 1.0;
    marker.action = 0;
	  marker.id = track_id;
	  marker.scale.x = 0.1;
	  marker.scale.y = 0.1;
	  marker.scale.z = 0.1;
	  geometry_msgs::Point point;
	  point.x = trackx;
	  point.y = tracky;
	  point.z = 0;

	  marker.points.push_back(point);
	  point.x = measx;
	  point.y = measy;
	  marker.points.push_back(point);

	  pub_.publish(marker);
	}
  

  void visualizer::visualize2DGates(double x, double y, Eigen::MatrixXd sk_inv, double tol, int id)
  {
  	// Gate: inv*Sk^-1*inv <= tolerance
  	// [x y] * | a b; c d | *[x y]^T = tol
  	// ax^2 = tol || dy^2 = tol

  	visualization_msgs::Marker marker;
  	marker.pose.orientation.x = 0;
  	marker.pose.orientation.y = 0;
  	marker.pose.orientation.z = 0;
  	marker.pose.orientation.w = 1;
  	marker.pose.position.z = 0;
  	marker.color.r = 0;
  	marker.color.g = 0.0;
  	marker.color.b = 1.0;
  	marker.color.a = 0.4;
  	marker.header.frame_id = "sensor";
  	marker.header.stamp = ros::Time::now();
    marker.action = 0;
    marker.type = 2; // cyclinder

	  marker.ns = "gates";
	  marker.id = id;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.scale.x =  2 * sqrt(tol*tol/sk_inv(0,0));
    marker.scale.y = 2 * sqrt(tol*tol/sk_inv(1,1));
    marker.scale.z = 0.1;
	  pub_.publish(marker);
  }

  void visualizer::visualizeArrays(std::vector<measurement> measurements)
  {
  	visualization_msgs::Marker marker;
    marker.ns = "raw_measurements";
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
    marker.type = 1;
    
    marker.action = 2; // Clear here
    pub_.publish(marker);
    
    marker.action = 0;
  	for(auto measurement : measurements)
  	{
  	  marker.id = count;
  	  marker.scale.x = measurement.d_;
  	  marker.scale.y = measurement.w_;
  	  marker.scale.z = measurement.h_ + 0.1;
  	  marker.pose.position.x = measurement.x_;
  	  marker.pose.position.y = measurement.y_;

  	  count++; // Using local ids;
  	  pub_.publish(marker);
  	}
  }

  void visualizer::visFilterStates(Eigen::VectorXd mean, Eigen::MatrixXd cov, std::string ns, std::string state)
  {
  	visualization_msgs::Marker marker;
  	int count = 0;
  	marker.pose.orientation.x = 0;
  	marker.pose.orientation.y = 0;
  	marker.pose.orientation.z = 0;
  	marker.pose.orientation.w = 1;
  	marker.pose.position.z = 0;
  	if(state == "initial")
  	{
  		marker.color.r = 1.0;
  		marker.color.g = 0.0;
  		marker.color.b = 0.0;
  		marker.color.a = 0.5;
  	}
  	else if(state == "new")
  	{
  		marker.color.r = 0.0;
  		marker.color.g = 1.0;
  		marker.color.b = 0.0;
  		marker.color.a = 0.5;
  	}
  	else if(state == "out_gates")
  	{
			marker.color.r = 0.0;
  		marker.color.g = 0.0;
  		marker.color.b = 1.0;
  		marker.color.a = 0.5;
  	}
  	marker.header.frame_id = "sensor";
  	marker.header.stamp = ros::Time::now();
    marker.action = 0;
    marker.type = 1;

    // Form position - size estimate
	marker.ns = std::string("track/") + ns;
	marker.id = 1;
	marker.scale.x = mean(4);
	marker.scale.y = mean(5);
	marker.scale.z = mean(6)+0.1;
	marker.pose.position.x = mean(0);
	marker.pose.position.y = mean(1);

	count++; // Using local ids;
	pub_.publish(marker);
	// Publish velocity estimate
	marker.type = 0;
	tf2::Quaternion quat_tf;
	quat_tf.setRPY(0, 0, atan2(mean(3),mean(2)));
	marker.pose.orientation = tf2::toMsg(quat_tf);
	marker.ns += std::string("/velocity");
	double vis_scale = 5;
	marker.scale.x = sqrt(mean(3)*mean(3)+mean(2)*mean(2)) * vis_scale;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	pub_.publish(marker);
	// Visualize belief?
  	}
};