#include "tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rm_auto_aim
{
Tracker::Tracker(double max_match_distance)
: tracker_state(LOST),
  measurement(Eigen::VectorXd::Zero(3)),
  target_state(Eigen::VectorXd::Zero(6)),
  max_match_distance_(max_match_distance),
  last_yaw_(0.0)
{
}

void Tracker::init(const Target::SharedPtr & target_msg)
{
  RCLCPP_DEBUG(rclcpp::get_logger("light_tracker"), "Init KF!");
  if (target_msg == nullptr) {
    return;
  }

  initKF(*target_msg);

  tracker_state = DETECTING;

  change_count_ = 0;
  change_thres = 20;
  RCLCPP_DEBUG(rclcpp::get_logger("light_tracker"), "Inited KF!");
}

void Tracker::update(const Target::SharedPtr & target_msg)
{
  // KF predict
  RCLCPP_DEBUG(rclcpp::get_logger("light_tracker"), "KF predict");
  Eigen::VectorXd kf_prediction = kf.predict(target_msg->dt);

  bool matched = false;
  // Use KF prediction as default target state if no matched light is found
  target_state = kf_prediction;
  if (target_msg != nullptr) {
    auto predicted_position = getLightPositionFromState(kf_prediction);
    double min_position_diff = DBL_MAX;

    // Calculate the difference between the predicted position and the current light position
    auto p = target_msg->position;
    Eigen::Vector3d position_vec(p.x, p.y, p.z);
    min_position_diff = (predicted_position - position_vec).norm();

    //TODO:add selection
    if (min_position_diff < max_match_distance_) {
      // Matched light found
      matched = true;
      // Update KF
      Eigen::VectorXd measurement(3);
      measurement << p.x, p.y, p.z;
      target_state = kf.update(measurement);
      RCLCPP_DEBUG(rclcpp::get_logger("light_tracker"), "KF update");
    }
  }

  if (!matched) {
    // No matched light found
    RCLCPP_WARN(rclcpp::get_logger("light_tracker"), "No matched light found!");
  }

  // Tracking state machine
  // RCLCPP_DEBUG(rclcpp::get_logger("light_tracker"), "Tracking state machine");
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
    // } else if (tracker_state == CHANGE_TARGET) {
    //   if (change_count_ > change_thres) {
    //     tracker_state = TRACKING;
    //     change_count_ = 0;
    //   } else {
    //     change_count_++;
    //   }
  }
  // RCLCPP_DEBUG(rclcpp::get_logger("light_tracker"), "KF predicted");
}

void Tracker::initKF(const Target & target_msg)
{
  double xl = target_msg.position.x;
  double yl = target_msg.position.y;
  double zl = target_msg.position.z;

  /*Todo:measure  a good initial position*/
  // Set initial position at ?m behind the target
  target_state = Eigen::VectorXd::Zero(6);

  target_state << xl, yl, zl, 0, 0, 0;

  kf.setState(target_state);
}

/*Todo:change the funtion*/
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get light yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getLightPositionFromState(const Eigen::VectorXd & x)
{
  // Calculate predicted position of the current light
  double xl = x(0), yl = x(1), zl = x(2);
  return Eigen::Vector3d(xl, yl, zl);
}

}  // namespace rm_auto_aim
