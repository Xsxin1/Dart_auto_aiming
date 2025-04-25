#ifndef DETECTOR__DETECTOR_NODE_HPP_
#define DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/debug_lights.hpp"
#include "auto_aim_interfaces/msg/lights.hpp"
#include "detector/detector.hpp"
#include "detector/pnp_solver.hpp"

namespace rm_auto_aim
{

class LightDetectorNode : public rclcpp::Node
{
public:
  struct ParamInfo
  {
    std::string name;
    int & value;
    int min;
    int max;
    std::string description = "";
  };

  LightDetectorNode(const rclcpp::NodeOptions & options);

private:
  std::unique_ptr<Detector> initDetector();

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void compressedImageCallback(sensor_msgs::msg::CompressedImage::SharedPtr compressed_msg);

  void handleImage(cv::Mat & img, std_msgs::msg::Header header);
  std::vector<Light> detectLights(cv::Mat & img, std_msgs::msg::Header header);

  void createDebugPublishers();
  void destroyDebugPublishers();

  //  task subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  bool is_aim_task_;
  void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);

  // Light Detector
  std::unique_ptr<Detector> detector_;

  // Detected lights publisher
  auto_aim_interfaces::msg::Lights lights_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Lights>::SharedPtr lights_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;

  // Debug information
  bool fg_debug_;
  bool cv_debug_;
  void paramCallback(const rcl_interfaces::msg::ParameterEvent & event);
  std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr event_callback_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> fg_debug_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cv_debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr debug_lights_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher result_img_pub_;
};

}  // namespace rm_auto_aim

#endif  // DETECTOR__DETECTOR_NODE_HPP_
