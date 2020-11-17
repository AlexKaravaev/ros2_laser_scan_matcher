/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#include "ros2_laser_scan_matcher/laser_scan_matcher.h"

namespace scan_tools
{
LaserScanMatcher::LaserScanMatcher() : initialized_(false), got_map_(false)
{
  // Initiate parameters
  map_to_odom_.setIdentity();

   RCLCPP_INFO(get_logger(), "Creating laser_scan_matcher");

  add_parameter("base_frame", rclcpp::ParameterValue(std::string("base_link")),
    "Which frame to use for the robot base");
  add_parameter("odom_frame", rclcpp::ParameterValue(std::string("odom")),
    "Which frame to use for the odom");
  add_parameter("map_frame", rclcpp::ParameterValue(std::string("map")),
    "Which frame to use for the map");
  add_parameter("kf_dist_linear", rclcpp::ParameterValue(0.10),
    "When to generate keyframe scan.");
  add_parameter("kf_dist_angular", rclcpp::ParameterValue(0.10* (M_PI/180.0)),
    "When to generate keyframe scan.");
  
  kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;
  double transform_publish_period;
  double tmp;
  
  add_parameter("resolution", rclcpp::ParameterValue(0.025),
    "Resolution of the laser.");
  add_parameter("transform_publish_period", rclcpp::ParameterValue(0.05),
    "");


  // CSM parameters - comments copied from algos.h (by Andrea Censi)
  add_parameter("max_angular_correction_deg", rclcpp::ParameterValue(45.0),
    "Maximum angular displacement between scansr.");

  add_parameter("max_linear_correction", rclcpp::ParameterValue(0.5),
    "Maximum translation between scans (m).");

  add_parameter("max_iterations", rclcpp::ParameterValue(10),
    "Maximum ICP cycle iterationsr.");

  add_parameter("epsilon_xy", rclcpp::ParameterValue(0.000001),
   "A threshold for stopping (m).");

  add_parameter("epsilon_theta", rclcpp::ParameterValue(0.000001),
    "A threshold for stopping (rad).");

  add_parameter("max_correspondence_dist", rclcpp::ParameterValue(0.3),
    "Maximum distance for a correspondence to be valid.");

  add_parameter("sigma", rclcpp::ParameterValue(0.010),
    "Noise in the scan (m).");

  add_parameter("use_corr_tricks", rclcpp::ParameterValue(1.0),
    "Use smart tricks for finding correspondences.");

  add_parameter("restart", rclcpp::ParameterValue(0),
    "Restart if error is over threshold.");

  add_parameter("restart_threshold_mean_error", rclcpp::ParameterValue(0.01),
    "Threshold for restarting.");

  add_parameter("restart_dt", rclcpp::ParameterValue(1.0),
   "Displacement for restarting. (m).");

  add_parameter("restart_dtheta", rclcpp::ParameterValue(1.0),
    "Displacement for restarting. (rad).");

  add_parameter("clustering_threshold", rclcpp::ParameterValue(0.25),
    "Max distance for staying in the same clustering.");

  add_parameter("orientation_neighbourhood", rclcpp::ParameterValue(20),
    "Number of neighbour rays used to estimate the orientation.");
  
  add_parameter("use_point_to_line_distance", rclcpp::ParameterValue(1),
    "If 0, it's vanilla ICP.");

  add_parameter("do_alpha_test", rclcpp::ParameterValue(0),
   " Discard correspondences based on the angles.");

  add_parameter("do_alpha_test_thresholdDeg", rclcpp::ParameterValue(20.0),
    "Discard correspondences based on the angles - threshold angle, in degrees.");

  add_parameter("outliers_maxPerc", rclcpp::ParameterValue(0.9),
    "Percentage of correspondences to consider: if 0.9, \
        always discard the top 10% of correspondences with more error");


  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  add_parameter("outliers_adaptive_order", rclcpp::ParameterValue(0.7),
    "");
  
  add_parameter("outliers_adaptive_mult", rclcpp::ParameterValue(2.0),
    "");

  // If you already have a guess of the solution, you can compute the polar
  // angle
  // of the points of one scan in the new position. If the polar angle is not a
  // monotone
  // function of the readings index, it means that the surface is not visible in
  // the
  // next position. If it is not visible, then we don't use it for matching.
  add_parameter("do_visibility_test", rclcpp::ParameterValue(0),
    "");
  
  add_parameter("outliers_remove_doubles", rclcpp::ParameterValue(1),
    "No two points in laser_sens can have the same corr.");
  
  add_parameter("do_compute_covariance", rclcpp::ParameterValue(0),
    "If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov");
  
  add_parameter("debug_verify_tricks", rclcpp::ParameterValue(0),
    " Checks that find_correspondences_tricks gives the right answer.");
  
  add_parameter("use_ml_weights", rclcpp::ParameterValue(0),
    "If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to \
         compute the incidence beta, and the factor (1/cos^2(beta)) used to weight the \
         correspondence.");

  add_parameter("use_sigma_weights", rclcpp::ParameterValue(0),
    " If 1, the field 'readings_sigma' in the second scan is used to weight the correspondence by 1/sigma^2");
    


  // State variables

  f2b_.setIdentity();
  f2b_kf_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  // Initialize output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;


  // Subscribers
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 5);
  scan_filter_ = new tf2::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&LaserScanMatcher::scanCallback, this, _1));

  timer_ = this->create_wall_timer(
      transform_publish_period, std::bind(&LaserScanMatcher::publishLoop, this));
}

LaserScanMatcher::~LaserScanMatcher()
{
  if (transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

void LaserScanMatcher::publishTransform()
{
  boost::mutex::scoped_lock(map_to_odom_mutex_);
  rclcpp::Duration tf_expiration = now() + rclcpp::Duration(0.05,0.0);
  tfB_->sendTransform(tf2::StampedTransform(map_to_odom_, now(), map_frame_, odom_frame_));
}

void LaserScanMatcher::publishLoop(double transform_publish_period)
{
  publishTransform();

}

bool LaserScanMatcher::getOdomPose(tf::Transform& odom_to_base_tf, const ros::Time& t)
{
  tf2::Stamped<tf2::Pose> ident(tf2::Transform(tf2::createQuaternionFromRPY(0, 0, 0), tf2::Vector3(0, 0, 0)), t,
                              base_frame_);
  tf2::Stamped<tf2::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch (tf2::TransformException e)
  {
    RCLCPP_WARN(get_logger(),"Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

  odom_to_base_tf = odom_pose;

  return true;
}

LocalizedRangeScan* LaserScanMatcher::addScan(LaserRangeFinder* laser, const sensor_msgs::LaserScan::ConstPtr& scan,
                                              const tf2::Transform& odom_to_base_tf)
{
  // Create a vector of doubles for karto
  std::vector<double> readings;

  if (lasers_inverted_[scan->header.frame_id])
  {
    for (std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin(); it != scan->ranges.rend(); ++it)
    {
      readings.push_back(*it);
    }
  }
  else
  {
    for (std::vector<float>::const_iterator it = scan->ranges.begin(); it != scan->ranges.end(); ++it)
    {
      readings.push_back(*it);
    }
  }

  // create localized range scan
  LocalizedRangeScan* range_scan = new LocalizedRangeScan(laser, readings);
  range_scan->SetOdometricPose(Pose2(odom_to_base_tf.getOrigin().x(), odom_to_base_tf.getOrigin().y(),
                                     tf2::getYaw(odom_to_base_tf.getRotation())));

  tf::Transform map_to_base_tf = map_to_odom_ * odom_to_base_tf;
  range_scan->SetCorrectedPose(
      Pose2(map_to_base_tf.getOrigin().x(), map_to_base_tf.getOrigin().y(), tf2::getYaw(map_to_base_tf.getRotation())));

  return range_scan;
}

void LaserScanMatcher::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

  // Check whether we know about this laser yet
  LaserRangeFinder* laser = getLaser(scan);

  if (!laser)
  {
    RCLCPP_WARN(get_logger(),"Failed to create laser device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  if (processScan(laser, scan))
  {
    map_to_odom_mutex_.lock();
    map_to_odom_ = f2b_kf_ * odom_to_base_tf.inverse();
    map_to_odom_mutex_.unlock();

  }
}

bool LaserScanMatcher::processScan(LaserRangeFinder* laser, const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if (!getOdomPose(odom_to_base_tf, scan->header.stamp))
    return false;

  // If first scan, cache the tf from base to the scanner
  if (!initialized_)
  {
    input_.min_reading = scan->range_min;
    input_.max_reading = scan->range_max;

    laserScanToLDP(scan, prev_ldp_scan_);

    LocalizedRangeScan* pScan = addScan(laser, scan, odom_to_base_tf);
    allScans_.push_back(pScan);

    initialized_ = true;

    return true;
  }

  LDP curr_ldp_scan;
  laserScanToLDP(scan, curr_ldp_scan);

  LocalizedRangeScan* pScan = addScan(laser, scan, odom_to_base_tf);

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan_->odometry[0] = 0.0;
  prev_ldp_scan_->odometry[1] = 0.0;
  prev_ldp_scan_->odometry[2] = 0.0;

  prev_ldp_scan_->estimate[0] = 0.0;
  prev_ldp_scan_->estimate[1] = 0.0;
  prev_ldp_scan_->estimate[2] = 0.0;

  prev_ldp_scan_->true_pose[0] = 0.0;
  prev_ldp_scan_->true_pose[1] = 0.0;
  prev_ldp_scan_->true_pose[2] = 0.0;

  input_.laser_ref = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // estimated change since last scan
  // the predicted change of the laser's position, in the laser frame

  tf::Transform pr_ch_l;
  pr_ch_l = (f2b_kf_ * base_to_laser_).inverse() * map_to_odom_ * odom_to_base_tf * base_to_laser_;

  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = tf2::getYaw(pr_ch_l.getRotation());

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // Scan matching - using point to line icp from CSM

  sm_icp(&input_, &output_);
  tf::Transform corr_ch;

  if (output_.valid)
  {
    // the correction of the laser's position, in the laser frame
    tf2::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    // the correction of the base's position, in the base frame
    corr_ch = base_to_laser_ * corr_ch_l * base_to_laser_.inverse();

    // update the pose in the world frame
    f2b_ = f2b_kf_ * corr_ch;
  }
  else
  {
    corr_ch.setIdentity();
    RCLCPP_WARN(get_logger(),"Error in scan matching");
  }

  // **** swap old and new

  if (newKeyframeNeeded(corr_ch))
  {
    // generate a keyframe
    ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    f2b_kf_ = f2b_;

    Pose2 corrected_pose;
    corrected_pose.SetX(f2b_kf_.getOrigin().x());
    corrected_pose.SetY(f2b_kf_.getOrigin().y());
    corrected_pose.SetHeading(tf2::getYaw(f2b_kf_.getRotation()));

    pScan->SetCorrectedPose(corrected_pose);
    allScans_.push_back(pScan);

    return true;
  }
  else
  {
    ld_free(curr_ldp_scan);
    delete pScan;

    return false;
  }
}

bool LaserScanMatcher::newKeyframeNeeded(const tf2::Transform& d)
{
  if (fabs(tf2::getYaw(d.getRotation())) > kf_dist_angular_)
    return true;

  double x = d.getOrigin().getX();
  double y = d.getOrigin().getY();
  if (x * x + y * y > kf_dist_linear_sq_)
    return true;

  return false;
}

void LaserScanMatcher::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan, LDP& ldp)
{
  unsigned int n = scan->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // Calculate position in laser frame
    double r = scan->ranges[i];
    if ((r > scan->range_min) && (r < scan->range_max))
    {
      // Fill in laser scan data
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }
    ldp->theta[i] = scan->angle_min + i * scan->angle_increment;
    ldp->cluster[i] = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n - 1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

LaserRangeFinder* LaserScanMatcher::getLaser(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Check whether we know about this laser yet
  if (lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf2::Stamped<tf2::Pose> ident;
    tf2::Stamped<tf2::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try
    {
      tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch (tf2::TransformException e)
    {
      RCLCPP_WARN(get_logger(),"Failed to compute laser pose, aborting initialization (%s)", e.what());
      return NULL;
    }

    double yaw = tf2::getYaw(laser_pose.getRotation());

    RCLCPP_INFO(get_logger(),"laser %s's pose wrt base: %.3f %.3f %.3f", scan->header.frame_id.c_str(), laser_pose.getOrigin().x(),
             laser_pose.getOrigin().y(), yaw);

    base_to_laser_ = laser_pose;

    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser
    // frame
    // if the point's z-value is <=0, it is upside-down

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf2::Vector3> up(v, scan->header.stamp, base_frame_);

    try
    {
      tf_.transformPoint(scan->header.frame_id, up, up);
      RCLCPP_DEBUG(get_logger(),"Z-Axis in sensor frame: %.3f", up.z());
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN(get_logger(),"Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse)
      RCLCPP_INFO(get_logger(),"laser is mounted upside-down");

    // Create a laser range finder device and copy in data from the first scan

    LaserRangeFinder* laser = LaserRangeFinder::CreateLaserRangeFinder();
    laser->SetRangeThreshold(12.0);  // for UTM-30LX
    laser->SetOffsetPose(Pose2(laser_pose.getOrigin().x(), laser_pose.getOrigin().y(), yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;
  }

  return lasers_[scan->header.frame_id];
}

void LaserScanMatcher::createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t)
{
  t.setOrigin(tf2::Vector3(x, y, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}


}  // namespace scan_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scan_tools::LaserScanMatcher>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}