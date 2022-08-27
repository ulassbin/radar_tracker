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
      trackers_.push_back(kalman_filter::kalmanFilter(i));
  }

  estimator::~estimator()
  {
  }

  int estimator::getClosestTracker(const measurement meas) // Nearest neighbour approach
  {
  	double dist = std::numeric_limits<float>::infinity();
  	int closest = 0; // Maybe init this -1?
  	double temp = 0;
  	for(int i = 0; i < 10; i++)
  	{
  	  temp = hypot(trackers_[i].mean_(0) - meas.x_, trackers_[i].mean_(1) - meas.y_);
  	  if(temp < dist)
  	  {
  	  	dist = temp;
  	  	closest = i;
  	  }
  	}
  	return closest;
  }

  std::map<int, std::vector<std::pair<int, double>>> estimator::getMatchesInGate(double PD)
  {
    // Two questions comes to mind
    // Is 2d position enough?
    // How about adding velocities and-or volume info?
    double PG = 1; // Large gate assumption similar to the paper.
    double dist_sq = 0;
    double pos_dist = 0;
    double gate_eps = 1;
    Eigen::VectorXd innovation(7), meas_eig(7);
    Eigen::MatrixXd sk_inv2(2,2);
    std::vector<std::pair<int,double>> gated_measurements;
    std::map<int, std::vector<std::pair<int, double>>> matches;
    std::pair<int ,double> temp_p;
    double gate_tol = 2.0;
   double C = 0.3; // C is expected number of false measurements per unit volume of the gate.

    for(int i = 0; i < trackers_.size(); i++)
    {
      gated_measurements.clear();
      kalman_filter::kalmanFilter track = trackers_[i]; // Getting a copy of tracker object
      track.prediction(); // Just updating the copy, not the element itself...
      sk_inv2 =  track.cov_p_.block<2,2>(1,1).inverse();
      uviz_.visualizeTwo2DGates(track.mean_p_(0), track.mean_p_(1), sk_inv2, gate_tol, i);
      for(int j = 0 ; j < measurements_.size(); j++)
      {
        meas_eig = common::toEigen(measurements_[j]);
        innovation = meas_eig - track.mean_p_;
        dist_sq = innovation.transpose() * track.cov_p_.inverse() * innovation; // Covariance scales down the dimensions
        // We can compute and visualize 2d elipsoids here. 
        pos_dist = (innovation.head(2).transpose() * sk_inv2 *innovation.head(2));
        ROS_INFO("Tracker %d , meas %i, dist %.2f, pose_dist %.2f",i,j,sqrt(dist_sq),sqrt(pos_dist));
        if(dist_sq < gate_eps) //
        {
          double gauss_val = exp(-dist_sq/2.0);//Just using 2d values here...
          gated_measurements.push_back(std::make_pair(j, gauss_val)); 
        }
        // bj = exp(dist_sq);
      }
      double ellipse_area = M_PI*(gate_tol/sk_inv2(0,0)*gate_tol/sk_inv2(1,1));
      double b0 = pow(2*M_PI,2/2)*(C*ellipse_area/(M_PI*1))*(1-PD*PG)/PD; // / denum ...
      gated_measurements.push_back(std::make_pair(-1, b0));
      matches[i] = gated_measurements; // Assign the gated measurements to track.
    }
    return matches; 
  }

  bool estimator::checkMatchValidity(std::vector<std::pair<int,double>> candidates, double& normalizer)
  {
    // Calculates normalizer for weights
    // Checks if correspondences is more probable than no match.
    double b0(0.1); // 1-PD for 0;
    double match_prob(0);
    for(auto pair : candidates)
    {
      if(pair.first == -1)
        b0 = pair.second;
      else
        match_prob += pair.second; // b1 + b2 + .. bn;
    }
    normalizer = 1.0/(b0+match_prob);
    ROS_INFO("No match %.2f, match %.2f",b0,match_prob);
    return (match_prob>b0); //
  }

  void estimator::messageCallback(const std_msgs::Float32MultiArray msg)
  {
    bool valid(false);
    double norm;
    double PD = 0.9;
  	measurements_ = common::parseMeasurements(msg); // Store to array could be better if processing freq is low.
    uviz_.visualizeArrays(measurements_);
    std::map<int, std::vector<std::pair<int, double>>> matches;
    if(!trackers_[0].first_)
      matches = getMatchesInGate(PD);
    else
    {
      for(int i = 0; i < trackers_.size(); i++)
      {
        trackers_[i].assign(measurements_[i]);
        trackers_[i].first_ = false;
        uviz_.visFilterStates(trackers_[i].mean_, trackers_[i].cov_, std::to_string(i));
      }
      return;
    }

    ROS_INFO("Matches size %d",matches.size());
    // Now trace the matches...
    for(int i = 0; i < trackers_.size(); i++)
    {
      trackers_[i].prediction();
      uviz_.visFilterStates(trackers_[i].mean_, trackers_[i].cov_, std::to_string(i));
    }
    for(int i = 0; i < matches.size(); i++)
    {
      ROS_INFO("Track %d has %d matches", i, matches[i].size());
       // Just perform prediction ... 
      checkMatchValidity(matches[i], norm);
      for(auto pair : matches[i])
        ROS_INFO("Match %i b%i probability %.2f",i, pair.first, pair.second*norm); // b-1 is no match probability
      if(matches[i].size() > 1) // checkMatchValidity(matches[i], norm)
      {
        // Form mixture measurement based on bi weights. 
        // one approach is to have measurements formed as 
        // no_match_weight*Prediction + w0*match_0 + w1*match_1 + .... (However refeeding prediction can cause issues?)
        // Thats why if match is so and so valid, lets consider re-calculating normalizer
        // Method Proposed in Sonar Tracking .. PDA paper we need to change prediction values and use the weights as it is?
        //normalizer = 1 / (1 - 0.1 * normalizer); // Removing the effect of no match from mixing - Can test it either way if one has time.
        trackers_[i].pdaUpdate(matches[i], measurements_, PD, norm);
      }
      else
      {
        ROS_INFO("Matches %d not valid",i);
      }
      uviz_.visFilterStates(trackers_[i].mean_, trackers_[i].cov_, std::to_string(i));
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