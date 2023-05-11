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

#undef min
#undef max

namespace scan_tools
{

  void LaserScanMatcher::add_parameter(
      const std::string &name, const rclcpp::ParameterValue &default_value,
      const std::string &description, const std::string &additional_constraints,
      bool read_only)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  LaserScanMatcher::LaserScanMatcher() : Node("laser_scan_matcher"), initialized_(false)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // Initiate parameters

    RCLCPP_INFO(get_logger(), "Creating laser_scan_matcher");
    add_parameter("publish_odom", rclcpp::ParameterValue(std::string("")),
                  "If publish odometry from laser_scan. Empty if not, otherwise name of the topic");
    add_parameter("publish_tf", rclcpp::ParameterValue(false),
                  " If publish tf odom->base_link");

    add_parameter("base_frame", rclcpp::ParameterValue(std::string("base_link")),
                  "Which frame to use for the robot base");
    add_parameter("odom_frame", rclcpp::ParameterValue(std::string("/robot_interface/odom_laser")),
                  "Which frame to use for the odom");
    add_parameter("map_frame", rclcpp::ParameterValue(std::string("map")),
                  "Which frame to use for the map");
    add_parameter("laser_frame", rclcpp::ParameterValue(std::string("laser")),
                  "Which frame to use for the laser");
    add_parameter("kf_dist_linear", rclcpp::ParameterValue(0.10),
                  "When to generate keyframe scan.");
    add_parameter("kf_dist_angular", rclcpp::ParameterValue(10.0 * (M_PI / 180.0)),
                  "When to generate keyframe scan.");
    add_parameter("laser_scan_topic", rclcpp::ParameterValue(std::string("/robot_interface/micro_scan_front/laser_scan")),
                  "name of the laser scan topic");

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

    add_parameter("use_corr_tricks", rclcpp::ParameterValue(1),
                  "Use smart tricks for finding correspondences.");

    add_parameter("restart", rclcpp::ParameterValue(0),
                  "Restart if error is over threshold.");

    add_parameter("restart_threshold_mean_error", rclcpp::ParameterValue(0.01),
                  "Threshold for restarting.");

    add_parameter("restart_dt", rclcpp::ParameterValue(1.0),
                  "Displacement for restarting. (m).");

    add_parameter("restart_dtheta", rclcpp::ParameterValue(0.1),
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

    add_parameter("laser_odom_srv_channel", rclcpp::ParameterValue(std::string{"~/enable_laser_odom"}),"enable node service channel");
    
    
    
    auto enable_laser_odom_service_channel = this->get_parameter("laser_odom_srv_channel").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    laser_frame_ = this->get_parameter("laser_frame").as_string();
    kf_dist_linear_ = this->get_parameter("kf_dist_linear").as_double();
    kf_dist_angular_ = this->get_parameter("kf_dist_angular").as_double();
    odom_topic_ = this->get_parameter("publish_odom").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    laser_scan_topic_ = this->get_parameter("laser_scan_topic").as_string();

    publish_odom_ = (odom_topic_ != "");
    kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

    input_.max_angular_correction_deg = this->get_parameter("max_angular_correction_deg").as_double();
    input_.max_linear_correction = this->get_parameter("max_linear_correction").as_double();
    input_.max_iterations = this->get_parameter("max_iterations").as_int();
    input_.epsilon_xy = this->get_parameter("epsilon_xy").as_double();
    input_.epsilon_theta = this->get_parameter("epsilon_theta").as_double();
    input_.max_correspondence_dist = this->get_parameter("max_correspondence_dist").as_double();
    input_.sigma = this->get_parameter("sigma").as_double();
    input_.use_corr_tricks = this->get_parameter("use_corr_tricks").as_int();
    input_.restart = this->get_parameter("restart").as_int();
    input_.restart_threshold_mean_error = this->get_parameter("restart_threshold_mean_error").as_double();
    input_.restart_dt = this->get_parameter("restart_dt").as_double();
    input_.restart_dtheta = this->get_parameter("restart_dtheta").as_double();
    input_.clustering_threshold = this->get_parameter("clustering_threshold").as_double();
    input_.orientation_neighbourhood = this->get_parameter("orientation_neighbourhood").as_int();
    input_.use_point_to_line_distance = this->get_parameter("use_point_to_line_distance").as_int();
    input_.do_alpha_test = this->get_parameter("do_alpha_test").as_int();
    input_.do_alpha_test_thresholdDeg = this->get_parameter("do_alpha_test_thresholdDeg").as_double();
    input_.outliers_maxPerc = this->get_parameter("outliers_maxPerc").as_double();
    input_.outliers_adaptive_order = this->get_parameter("outliers_adaptive_order").as_double();
    input_.outliers_adaptive_mult = this->get_parameter("outliers_adaptive_mult").as_double();
    input_.do_visibility_test = this->get_parameter("do_visibility_test").as_int();
    input_.outliers_remove_doubles = this->get_parameter("outliers_remove_doubles").as_int();
    input_.do_compute_covariance = this->get_parameter("do_compute_covariance").as_int();
    input_.debug_verify_tricks = this->get_parameter("debug_verify_tricks").as_int();
    input_.use_ml_weights = this->get_parameter("use_ml_weights").as_int();
    input_.use_sigma_weights = this->get_parameter("use_sigma_weights").as_int();

    double transform_publish_period;
    double tmp;

    // State variables

    f2b_.setIdentity();
    prev_f2b_.setIdentity();
    f2b_kf_.setIdentity();
    input_.laser[0] = 0.0;
    input_.laser[1] = 0.0;
    input_.laser[2] = 0.0;

    // Initialize output_ vectors as Null for error-checking
    output_.cov_x_m = 0;
    output_.dx_dy1_m = 0;
    output_.dx_dy2_m = 0;

    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (publish_tf_)
      tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    if (publish_odom_)
    {
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SystemDefaultsQoS());
    }

    // Create services
    enable_node_srv_ = this->create_service<std_srvs::srv::SetBool>(
        enable_laser_odom_service_channel, bind(&LaserScanMatcher::subscribeToTopicsCb, this,
                                       std::placeholders::_1, std::placeholders::_2));
  }

  LaserScanMatcher::~LaserScanMatcher()
  {
  }


  void LaserScanMatcher::subscribeToTopicsCb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      // Subscribers
      this->scan_filter_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic_, rclcpp::SensorDataQoS(), std::bind(&LaserScanMatcher::scanCallback, this, std::placeholders::_1));
    }
    else
    {
      this->scan_filter_sub_.reset();
    }
    response->success = true;
  }

  void LaserScanMatcher::createCache(const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
  {
    a_cos_.clear();
    a_sin_.clear();

    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      a_cos_.push_back(cos(angle));
      a_sin_.push_back(sin(angle));
    }

    input_.min_reading = scan_msg->range_min;
    input_.max_reading = scan_msg->range_max;
  }

  void LaserScanMatcher::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)

  {

    if (!initialized_)
    {
      createCache(scan_msg); // caches the sin and cos of all angles

      // cache the static tf from base to laser
      if (!getBaseToLaserTf(laser_frame_))
      {
        RCLCPP_WARN(get_logger(), "Skipping scan");
        return;
      }

      laserScanToLDP(scan_msg, prev_ldp_scan_);
      last_icp_time_ = scan_msg->header.stamp;
      initialized_ = true;
    }

    LDP curr_ldp_scan;
    laserScanToLDP(scan_msg, curr_ldp_scan);
    processScan(curr_ldp_scan, scan_msg->header.stamp);
  }

  bool LaserScanMatcher::getBaseToLaserTf(const std::string &frame_id)
  {
    rclcpp::Time t = now();

    tf2::Stamped<tf2::Transform> base_to_laser_tf;
    geometry_msgs::msg::TransformStamped laser_pose_msg;
    try
    {
      laser_pose_msg = tf_buffer_->lookupTransform(base_frame_, frame_id, t, rclcpp::Duration(10, 0));
      base_to_laser_tf.setOrigin(tf2::Vector3(laser_pose_msg.transform.translation.x,
                                              laser_pose_msg.transform.translation.y,
                                              laser_pose_msg.transform.translation.z));
      tf2::Quaternion q(laser_pose_msg.transform.rotation.x,
                        laser_pose_msg.transform.rotation.y,
                        laser_pose_msg.transform.rotation.z,
                        laser_pose_msg.transform.rotation.w);
      base_to_laser_tf.setRotation(q);
    }
    catch (tf2::TransformException ex)
    {
      RCLCPP_INFO(get_logger(), "Could not get initial transform from base to laser frame, %s", ex.what());
      return false;
    }

    base_to_laser_ = base_to_laser_tf;
    laser_to_base_ = base_to_laser_.inverse();

    return true;
  }

  bool LaserScanMatcher::processScan(LDP &curr_ldp_scan, const rclcpp::Time &time)
  {

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

    tf2::Transform pr_ch_l;

    double dt = (now() - last_icp_time_).nanoseconds() / 1e+9;
    double pr_ch_x, pr_ch_y, pr_ch_a;

    // the predicted change of the laser's position, in the fixed frame

    tf2::Transform pr_ch;
    createTfFromXYTheta(0.0, 0.0, 0.0, pr_ch);

    // account for the change since the last kf, in the fixed frame

    pr_ch = pr_ch * (f2b_ * f2b_kf_.inverse());

    // the predicted change of the laser's position, in the laser frame

    pr_ch_l = laser_to_base_ * f2b_.inverse() * pr_ch * f2b_ * base_to_laser_;
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
    tf2::Transform corr_ch;

    if (output_.valid)
    {
      // the correction of the laser's position, in the laser frame
      tf2::Transform corr_ch_l;
      createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

      // the correction of the base's position, in the base frame
      corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

      // update the pose in the world frame
      f2b_ = f2b_kf_ * corr_ch;
    }

    else
    {
      corr_ch.setIdentity();
      RCLCPP_WARN(get_logger(), "Error in scan matching");
      return false;
    }

    if (publish_odom_)
    {
      // stamped Pose message
      nav_msgs::msg::Odometry odom_msg;

      odom_msg.header.stamp = time;
      odom_msg.header.frame_id = odom_frame_;
      odom_msg.child_frame_id = base_frame_;
      odom_msg.pose.pose.position.x = f2b_.getOrigin().x();
      odom_msg.pose.pose.position.y = f2b_.getOrigin().y();
      odom_msg.pose.pose.position.z = f2b_.getOrigin().z();

      odom_msg.pose.pose.orientation.x = f2b_.getRotation().x();
      odom_msg.pose.pose.orientation.y = f2b_.getRotation().y();
      odom_msg.pose.pose.orientation.z = f2b_.getRotation().z();
      odom_msg.pose.pose.orientation.w = f2b_.getRotation().w();

      // Get pose difference in base frame and calculate velocities
      auto pose_difference = prev_f2b_.inverse() * f2b_;
      odom_msg.twist.twist.linear.x = pose_difference.getOrigin().getX() / dt;
      odom_msg.twist.twist.linear.y = pose_difference.getOrigin().getY() / dt;
      odom_msg.twist.twist.angular.z = tf2::getYaw(pose_difference.getRotation()) / dt;

      prev_f2b_ = f2b_;

      odom_publisher_->publish(odom_msg);
    }

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.transform.translation.x = f2b_.getOrigin().x();
      tf_msg.transform.translation.y = f2b_.getOrigin().y();
      tf_msg.transform.translation.z = f2b_.getOrigin().z();
      tf_msg.transform.rotation.x = f2b_.getRotation().x();
      tf_msg.transform.rotation.y = f2b_.getRotation().y();
      tf_msg.transform.rotation.z = f2b_.getRotation().z();
      tf_msg.transform.rotation.w = f2b_.getRotation().w();

      tf_msg.header.stamp = time;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      // tf2::Stamped<tf2::Transform> transform_msg (f2b_, time, map_frame_, base_frame_);
      tfB_->sendTransform(tf_msg);
    }

    // **** swap old and new
    if (newKeyframeNeeded(corr_ch))
    {
      // generate a keyframe
      ld_free(prev_ldp_scan_);
      prev_ldp_scan_ = curr_ldp_scan;
      f2b_kf_ = f2b_;
    }
    else
    {
      ld_free(curr_ldp_scan);
    }
    last_icp_time_ = now();
    return true;
  }

  bool LaserScanMatcher::newKeyframeNeeded(const tf2::Transform &d)
  {
    if (fabs(tf2::getYaw(d.getRotation())) > kf_dist_angular_)
      return true;

    double x = d.getOrigin().getX();
    double y = d.getOrigin().getY();
    if (x * x + y * y > kf_dist_linear_sq_)
      return true;

    return false;
  }

  void LaserScanMatcher::laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr &scan, LDP &ldp)
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
        ldp->readings[i] = -1; // for invalid range
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

  void LaserScanMatcher::createTfFromXYTheta(double x, double y, double theta, tf2::Transform &t)
  {
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);
  }

} // namespace scan_tools

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scan_tools::LaserScanMatcher>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
