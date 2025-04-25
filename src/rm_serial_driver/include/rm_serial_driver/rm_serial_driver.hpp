// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// #include "auto_aim_interfaces/msg/target.hpp"
// #include "auto_aim_interfaces/msg/time_info.hpp"

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendLightData(const std_msgs::msg::Float64::ConstSharedPtr msg);

  void reopenPort();

  void setParam(const rclcpp::Parameter & param);

  // void resetTracker();

  // void changeTarget();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // Param client to set detect_colr
  // using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  // bool initial_set_param_ = false;
  // uint8_t previous_receive_color_ = 0;
  // rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  // ResultFuturePtr set_param_future_;

  // Service client to reset tracker
  // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Service client to change target
  // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_target_client_;

  // Aimimg point receiving from serial port for visualization
  // visualization_msgs::msg::Marker aiming_point_;

  // Broadcast tf from odom to gimbal_link
  // double timestamp_offset_ = 0;
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr aim_sub_;

  // message_filters::Subscriber<auto_aim_interfaces::msg::Target> aim_sub_;
  // message_filters::Subscriber<auto_aim_interfaces::msg::TimeInfo> aim_time_info_sub_;

  // typedef message_filters::sync_policies::ApproximateTime<
  //   auto_aim_interfaces::msg::Target, auto_aim_interfaces::msg::TimeInfo>
  //   aim_syncpolicy;
  // typedef message_filters::Synchronizer<aim_syncpolicy> AimSync;
  // std::shared_ptr<AimSync> aim_sync_;

  // For debug usage
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::thread receive_thread_;

  // Task message
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;

  // Time message
  // rclcpp::Publisher<auto_aim_interfaces::msg::TimeInfo>::SharedPtr aim_time_info_pub_;

  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr record_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr light_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr light_sub_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
