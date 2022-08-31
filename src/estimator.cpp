#include <radar_estimator/estimator.h>

namespace estimator
{
  estimator::estimator()
  {
  	ros::NodeHandle nh;
  	got_msg_ = false;
  	sub_msgs_ =  nh.subscribe("/lrrObjects", 10, &estimator::messageCallback, this);
    pub_debug_ = nh.advertise<radar_estimator::EstimatorDebug>("estimator_debug", 10, false);
    //timer_ = private_nh_.createTimer(ros::Duration(0.1), std::bind(&estimator::timerCallback, this, _1));
    //for(int i = 0; i<10; i++) // Don't initialize yet...
      //trackers_.push_back(kalman_filter::kalmanFilter(i));
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
    double gate_eps = 2;
    Eigen::VectorXd innovation(7), meas_eig(7);
    Eigen::MatrixXd sk_inv2(2,2);
    std::vector<std::pair<int,double>> gated_measurements;
    std::map<int, std::vector<std::pair<int, double>>> matches;
    std::pair<int ,double> temp_p;
    double C = 0.3; // C is expected number of false measurements per unit volume of the gate.

    for(int i = 0; i < trackers_.size(); i++)
    {
      gated_measurements.clear();
      kalman_filter::kalmanFilter track = trackers_[i]; // Getting a copy of tracker object
      sk_inv2 =  track.cov_p_.block<2,2>(1,1).inverse();
      uviz_.visualize2DGates(track.mean_p_(0), track.mean_p_(1), sk_inv2, gate_eps, i);
      for(int j = 0; j < measurements_.size(); j++)
      {
        meas_eig = common::toEigen(measurements_[j]);
        innovation = meas_eig - track.mean_p_;
        //dist_sq = innovation.transpose() * track.cov_p_.inverse() * innovation; // Covariance scales down the dimensions
        // We can compute and visualize 2d elipsoids here. 
        pos_dist = (innovation.head(2).transpose() * sk_inv2 * innovation.head(2));
        ROS_INFO("Tracker %d , meas %i, pose_dist %.2f",i,j,sqrt(pos_dist));
        if(sqrt(pos_dist) < gate_eps) //
        {
          double gauss_val = exp(-pos_dist/2.0);//Just using 2d values here...
          gated_measurements.push_back(std::make_pair(j, gauss_val)); 
        }
        // bj = exp(dist_sq);
      }
      double ellipse_area = M_PI*(gate_eps/sk_inv2(0,0)*gate_eps/sk_inv2(1,1));
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
    ROS_DEBUG("No match %.2f, match %.2f",b0,match_prob);
    return (match_prob>b0); //
  }

  void estimator::broadcastDebugMsg()
  {
    debug_.header.stamp = ros::Time::now();
    debug_.header.frame_id = "sensor";
    debug_.n_filter_active = trackers_.size();
    //debug_.n_filter_deleted ...
    //debug_.n_filter_recreated ...
    debug_.filters.clear();
    for(auto filt : trackers_)
      debug_.filters.push_back(filt.msg_);
    pub_debug_.publish(debug_);
  }

  void estimator::visAllFilters()
  {
    for(int i = 0; i < trackers_.size(); i++)
      uviz_.visFilterStates(trackers_[i].mean_, trackers_[i].cov_, std::to_string(i), trackers_[i].state_);
  }

  void estimator::messageCallback(const std_msgs::Float32MultiArray msg)
  {
    bool valid(false);
    double norm;
    double PD = 0.9;
  	measurements_ = common::parseMeasurements(msg); // Store to array could be better if processing freq is low.
    uviz_.visualizeArrays(measurements_);
    std::map<int, std::vector<std::pair<int, double>>> matches;
    if(trackers_.size() == 0)
    {
      for(auto meas : measurements_)
      {
        // Initializing a set of filters
        trackers_.push_back(kalman_filter::kalmanFilter(trackers_.size()));
        trackers_.back().assign(meas); // Assign to new element
      }
      visAllFilters();
    }
    else 
    {
      // We have new objects
      // Get the gated objects, assign the other ones.
      for(int i = 0; i < trackers_.size(); i++)
      {
        trackers_[i].prediction();
      }
      visAllFilters();
      matches = getMatchesInGate(PD);
      
      std::vector<int> assigned_matches;
      ROS_INFO("MEAS size %d", measurements_.size());
      for(auto match : matches)
      { // Match is std::pair<int, vector<pair<match_id, gauss_val>>;
        for(std::pair<int,double> match_item : match.second) // NO match index
        {
          if(match_item.first == -1)
            continue;
          assigned_matches.push_back(match_item.first);
          ROS_INFO("Track %d Gated %d at position %.2f %.2f", match.first, match_item.first, measurements_[match_item.first].x_, measurements_[match_item.first].y_);
        }
      }
      // Update assigned filters
      for(int i = 0; i < matches.size(); i++)
      {
        ROS_DEBUG("Track %d has %d matches", i, matches[i].size());
         // Just perform prediction ... 
        checkMatchValidity(matches[i], norm);
        for(auto pair : matches[i])
          ROS_DEBUG("Match %i b%i probability %.2f",i, pair.first, pair.second*norm); // b-1 is no match probability
        if(matches[i].size() > 1) // checkMatchValidity(matches[i], norm)
        {
          // Form mixture measurement based on bi weights. 
          trackers_[i].pdaUpdate(matches[i], measurements_, PD, norm);
        }
        else
        {
          ROS_INFO("Matches %d not valid",i);
        }
      }

      // Create not gated messages.
      std::vector<int>::iterator it;
      for(int meas_id = 0; meas_id < measurements_.size(); meas_id++)
      {
        it = std::find(assigned_matches.begin(), assigned_matches.end(), meas_id);
        if(it == assigned_matches.end()) // IF ID is not found
        { 
          // New element Construct & assign
          std::string cons_reason = (trackers_.size() > measurements_.size()?"out_gates":"new");
          ROS_INFO("Couldn't find match for measurement %d at %.2f, %.2f", meas_id, measurements_[meas_id].x_, measurements_[meas_id].y_);
          trackers_.push_back(kalman_filter::kalmanFilter(trackers_.size(), cons_reason)); // Lets color these different
          trackers_.back().assign(measurements_[meas_id]); // Assign to new element
        }
      }
    }
    visAllFilters();
    broadcastDebugMsg(); // Broadcast after visualization
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