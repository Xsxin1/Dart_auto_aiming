#include <ros2_hik_camera/openvino.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sstream>

// opencv
#include <opencv2/opencv.hpp>

// ros
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "MvCameraControl.h"

using namespace std;
using namespace cv;
using namespace std::chrono;

// #define GUI

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  /*****************************HIK CAMERA***************************/
  // 构造函数，初始化节点
  explicit HikCameraNode(const rclcpp::NodeOptions & options);

  // 析构函数，销毁节点
  ~HikCameraNode() override;

private:
  // 主函数
  void processImage(cv::Mat & rgb_image);
  void init();

  /*****************************KALMAN FILTER***************************/
  void kalmanInit();
  cv::Point2f kalmanUpdate(cv::Point2f measured_pos, double current_timestamp);
  cv::Point2f kalmanPredictOnly(double current_timestamp);

  cv::KalmanFilter kf;
  bool kf_initialized;
  cv::Mat measurement;
  double last_timestamp;
  int lost_count_;
  std_msgs::msg::Header header;

  /*****************************OPENVINO***************************/
  cv::Point2f detectYOLO(cv::Mat & origin, cv::Mat & viz, float size);

  YOLO_OPENVINO yolo_openvino;
  ov::CompiledModel model;
  /*****************************ROS***************************/
  // subscribers

  // publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dyaw_pub_;

  // callbacks

  // visualization
  image_transport::Publisher roi_pub_;
  image_transport::Publisher det_pub_;
  image_transport::Publisher res_pub_;
  void createDebugPub();
  void destroyDebugPub();

  // param
  std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr event_callback_handle_;

  void declareParams();
  void paramsEventCB(const rcl_interfaces::msg::ParameterEvent & event);

  /*****************************HIK CAMERA***************************/
  // 声明参数
  void declareParameters();

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  // 参数回调函数
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // 图像数据缓冲区
  std::vector<unsigned char> image_buffer_;
  // 相机返回值

  int nRet = MV_OK;
  void * camera_handle_;
  // 图像信息
  MV_IMAGE_BASIC_INFO img_info_;

  // 像素转换参数
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  // 失败次数
  int fail_count_ = 0;
  // 捕获线程
  std::thread capture_thread_;

  /*****************************HIK CAMERA***************************/
};
}  // namespace hik_camera
