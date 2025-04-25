#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "detector/detector.hpp"
#include "detector/detector_node.hpp"
#include "detector/light.hpp"
#include "detector/pnp_solver.hpp"

namespace rm_auto_aim
{
LightDetectorNode::LightDetectorNode(const rclcpp::NodeOptions & options)
: Node("light_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // Lights Publisher
  lights_pub_ = this->create_publisher<auto_aim_interfaces::msg::Lights>(
    "detector/lights", rclcpp::SensorDataQoS());

  // Debug Publishers
  fg_debug_ = this->declare_parameter("fg_detector_debug_", true);
  cv_debug_ = this->declare_parameter("cv_detector_debug_", false);

  if (fg_debug_) {
    createDebugPublishers();
  }

  // Debug param change moniter
  param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  fg_debug_cb_handle_ =
    param_sub_->add_parameter_callback("fg_detector_debug_", [this](const rclcpp::Parameter & p) {
      fg_debug_ = p.as_bool();
      fg_debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });
  cv_debug_cb_handle_ = param_sub_->add_parameter_callback(
    "cv_detector_debug_", [this](const rclcpp::Parameter & p) { cv_debug_ = p.as_bool(); });
  event_callback_handle_ = param_sub_->add_parameter_event_callback(
    std::bind(&LightDetectorNode::paramCallback, this, std::placeholders::_1));

  // Detector
  detector_ = initDetector();

  // Task subscriber
  task_sub_ = this->create_subscription<std_msgs::msg::String>(
    "task_mode", 10, std::bind(&LightDetectorNode::taskCallback, this, std::placeholders::_1));

  // Camera Info subscriber
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });

  // image subscriber()
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::SensorDataQoS(),
    std::bind(&LightDetectorNode::imageCallback, this, std::placeholders::_1));

  // compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
  //   "image_raw/compressed", 10,
  //   std::bind(&LightDetectorNode::compressedImageCallback, this, std::placeholders::_1));
}

std::unique_ptr<Detector> LightDetectorNode::initDetector()
{
  Detector::LightParams l_params = {
    .binaryThreshold = 7,
    .epsilon = 1000,
    .contoursRatioMin = 50,   // width/height
    .contoursRatioMax = 150,  // width/height
    .contoursAreaMin = 0,
    .contoursAreaMax = 20000,

    .filledRatioMin = 5,  // filled/area
    .filledRatioMax = 60,
    .ellipseAreaMin = 0,      // width*height
    .ellipseAreaMax = 10000,  // width*height
    .ellipseRatioMin = 0,     // width/height
    .ellipseRatioMax = 150,   // width/height
    .detectColor = RED};

  //声明参数
  std::vector<ParamInfo> params = {
    {"binary_thres", l_params.binaryThreshold, 0, 255},
    {"epsilon", l_params.epsilon, 0, 10000},
    {"contours_ratio_min", l_params.contoursRatioMin, 0, 200},
    {"contours_ratio_max", l_params.contoursRatioMax, 0, 200},
    {"contours_area_max", l_params.contoursAreaMax, 0, 20000},
    {"contours_area_min", l_params.contoursAreaMin, 0, 20000},
    {"ellipse_area_max", l_params.ellipseAreaMax, 0, 10000},
    {"ellipse_area_min", l_params.ellipseAreaMin, 0, 10000},
    {"ellipse_ratio_min", l_params.ellipseRatioMin, 0, 200},
    {"ellipse_ratio_max", l_params.ellipseRatioMax, 0, 200},
    {"filled_ratio_min", l_params.filledRatioMin, 0, 100},
    {"filled_ratio_max", l_params.filledRatioMax, 0, 100},
    {"detect_color", l_params.detectColor, 0, 1, "0-RED, 1-BLUE"}};

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  for (auto & param : params) {
    param_desc.description = param.description;
    param_desc.integer_range[0].from_value = param.min;
    param_desc.integer_range[0].to_value = param.max;
    param.value = this->declare_parameter(param.name, param.value, param_desc);
  }

  RCLCPP_INFO(this->get_logger(), "cv_debug_ = %d", cv_debug_);
  auto detector = std::make_unique<Detector>(l_params, cv_debug_, fg_debug_);

  return detector;
}

void LightDetectorNode::paramCallback(const rcl_interfaces::msg::ParameterEvent & event)
{
  static const std::map<std::string, int *> param_mapping = {
    {"binary_thres", &detector_->l.binaryThreshold},
    {"epsilon", &detector_->l.epsilon},
    {"contours_ratio_min", &detector_->l.contoursRatioMin},
    {"contours_ratio_max", &detector_->l.contoursRatioMax},
    {"contours_area_max", &detector_->l.contoursAreaMax},
    {"contours_area_min", &detector_->l.contoursAreaMin},
    {"ellipse_area_max", &detector_->l.ellipseAreaMax},
    {"ellipse_area_min", &detector_->l.ellipseAreaMin},
    {"filled_ratio_min", &detector_->l.filledRatioMin},
    {"filled_ratio_max", &detector_->l.filledRatioMax},
    {"detect_color", &detector_->l.detectColor}};

  for (const auto & changed_param : event.changed_parameters) {
    auto it = param_mapping.find(changed_param.name);
    if (it != param_mapping.end()) {
      *(it->second) = changed_param.value.integer_value;
    }
  }
}

//TODO:add model switch
void LightDetectorNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg)
{
  std::string task_mode = task_msg->data;
}

void LightDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  handleImage(img, img_msg->header);
}

void LightDetectorNode::compressedImageCallback(
  sensor_msgs::msg::CompressedImage::SharedPtr img_msg)
{
  img_msg->header.stamp = this->now();
  auto img = cv::imdecode(cv::Mat(img_msg->data), cv::IMREAD_COLOR);
  handleImage(img, img_msg->header);
}

void LightDetectorNode::handleImage(cv::Mat & img, std_msgs::msg::Header header)
{
  auto lights = detectLights(img, header);
  if (pnp_solver_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PnP solver not initialized!");
    return;
  }

  lights_msg_.header = header;
  lights_msg_.lights.clear();
  auto_aim_interfaces::msg::Light light_msg;
  for (auto & light : lights) {
    cv::Mat rvec, tvec;

    if (!pnp_solver_->solvePnP(light, rvec, tvec)) {
      RCLCPP_WARN(this->get_logger(), "PnP solver fail!");
      continue;
    }

    //TODO: add light type
    light.distance = sqrt(
      tvec.at<double>(0, 0) * tvec.at<double>(0, 0) +
      tvec.at<double>(1, 0) * tvec.at<double>(1, 0) +
      tvec.at<double>(2, 0) * tvec.at<double>(2, 0));
    if (cv_debug_) {
      cv::ellipse(img, lights[0], cv::Scalar(0, 10, 255), 6);
      cv::circle(img, lights[0].center, 5, cv::Scalar(0, 10, 255), 6);
      cv::putText(
        img, std::to_string(light.distance),
        cv::Point2f(
          lights[0].center.x - lights[0].size.width / 2,
          lights[0].center.y - lights[0].size.height / 2),
        cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 255), 6);
    }

    // Fill light_msg
    // Fill pose
    light_msg.pose.position.x = tvec.at<double>(0);
    light_msg.pose.position.y = tvec.at<double>(1);
    light_msg.pose.position.z = tvec.at<double>(2);

    // rvec to 3x3 rotation matrix
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    // rotation matrix to quaternion
    tf2::Matrix3x3 tf2_rotation_matrix(
      rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
      rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
      rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
      rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
      rotation_matrix.at<double>(2, 2));
    tf2::Quaternion tf2_q;
    tf2_rotation_matrix.getRotation(tf2_q);
    light_msg.pose.orientation = tf2::toMsg(tf2_q);

    //fill distance
    light_msg.distance = light.distance;

    lights_msg_.lights.emplace_back(light_msg);
  }

  // Publishing detected lights
  lights_pub_->publish(lights_msg_);

  // CV Visualization
  if (cv_debug_) {
    cv::resize(img, img, cv::Size(1280, 720));
    cv::imshow("vis_light", img);
    cv::waitKey(1);
  }
}

std::vector<Light> LightDetectorNode::detectLights(cv::Mat & img, std_msgs::msg::Header header)
{
  std::vector<Light> lights;
  lights = detector_->detect(img);

  auto final_time = this->now();
  auto latency = (final_time - header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_detector"), "Latency: " << latency << "ms");

  // Publish debug info
  if (fg_debug_) {
    binary_img_pub_.publish(
      cv_bridge::CvImage(header, "mono8", detector_->binary_img).toImageMsg());
    // std::cout << detector_->debug_lights_msg_.data.size() << std::endl;
    debug_lights_pub_->publish(detector_->debug_lights_msg_);

    detector_->drawResults(img);
    // Draw camera center
    cv::line(
      img, cv::Point(cam_center_.x + 50, cam_center_.y + 80),
      cv::Point(cam_center_.x + 50, cam_center_.y - 80), cv::Scalar(0, 255, 0), 4);
    cv::line(
      img, cv::Point(cam_center_.x - 50, cam_center_.y + 80),
      cv::Point(cam_center_.x - 50, cam_center_.y - 80), cv::Scalar(0, 255, 0), 4);
    cv::line(
      img, cv::Point(cam_center_.x + 90, cam_center_.y),
      cv::Point(cam_center_.x - 90, cam_center_.y), cv::Scalar(0, 255, 0), 4);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    result_img_pub_.publish(cv_bridge::CvImage(header, "bgr8", img).toImageMsg());
  }

  return lights;
}

void LightDetectorNode::createDebugPublishers()
{
  debug_lights_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("detector/debug_lights", 10);
  binary_img_pub_ = image_transport::create_publisher(this, "detector/binary_img");
  result_img_pub_ = image_transport::create_publisher(this, "detector/result_img");
}

void LightDetectorNode::destroyDebugPublishers()
{
  debug_lights_pub_.reset();
  binary_img_pub_.shutdown();
  result_img_pub_.shutdown();
}
}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::LightDetectorNode)
