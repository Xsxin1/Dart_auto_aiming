#ifndef TRACKER__TRACKER_HPP_
#define TRACKER__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "auto_aim_interfaces/msg/lights.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "tracker/kalman_filter.hpp"

namespace rm_auto_aim
{

class Tracker
{
public:
  Tracker(double max_match_distance);

  using Lights = auto_aim_interfaces::msg::Lights;
  using Light = auto_aim_interfaces::msg::Light;
  using Target = auto_aim_interfaces::msg::Target;

  void init(const Target::SharedPtr & target_msg);

  void update(const Target::SharedPtr & target_msg);

  KalmanFilter kf;

  int tracking_thres;
  int lost_thres;
  int change_thres;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
    CHANGE_TARGET,
  } tracker_state;

  Light tracked_light;

  double info_position_diff;
  double info_yaw_diff;

  Eigen::VectorXd measurement;

  Eigen::VectorXd target_state;

  Eigen::Vector3d prior_position;

private:
  void initKF(const Target & target_msg);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  Eigen::Vector3d getLightPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_;

  double max_match_yaw_diff_;

  int detect_count_;
  int lost_count_;
  int change_count_;

  double last_yaw_;
};

}  // namespace rm_auto_aim

#endif  // TRACKER__TRACKER_HPP_
