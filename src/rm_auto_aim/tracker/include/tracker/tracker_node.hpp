#ifndef TRACKER__TRACKER_NODE_HPP_
#define TRACKER__TRACKER_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/lights.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "tracker/tracker.hpp"
// #include "auto_aim_interfaces/msg/tracker_info.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Lights>;
class LightTrackerNode : public rclcpp::Node
{
public:
  explicit LightTrackerNode(const rclcpp::NodeOptions & options);

private:
  void lightsCallback(const auto_aim_interfaces::msg::Lights::SharedPtr lights_ptr);

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  // // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_;

  // light tracker
  double q_pos, q_vel, r_pos;
  double max_armor_distance_;
  double lost_time_thres_;
  int tracking_thres_;
  std::unique_ptr<Tracker> tracker_;
  std::shared_ptr<auto_aim_interfaces::msg::Target> target_msg_;

  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  // Change target service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_target_srv_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Lights> lights_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker linear_v_marker_;  //线速度
  visualization_msgs::msg::Marker light_marker_;     //引导灯预测位置
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // TRACKER__TRACKER_NODE_HPP_
