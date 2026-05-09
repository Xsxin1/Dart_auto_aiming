#include "ros2_hik_camera/hik_camera_node.h"

namespace hik_camera
{

void HikCameraNode ::processImage(cv::Mat &rgb_image)
{
  auto start = std::chrono::steady_clock::now();

  header.stamp = this->now();
  header.frame_id = "cam";

  cv::Mat bgr_img, viz;
  cv::cvtColor(rgb_image, bgr_img, cv::COLOR_RGB2BGR);
  cv::cvtColor(rgb_image, viz, cv::COLOR_RGB2BGR);

  auto target = detectYOLO(bgr_img, viz, 320);

  double current_timestamp = header.stamp.sec + header.stamp.nanosec * 1e-9;

  cv::Point2f filtered_center(-1, -1);
  if (target.x != -1)
  {
    filtered_center = kalmanUpdate(target, current_timestamp);
    lost_count_ = 0;
  }
  else
  {
    if (kf_initialized)
    {
      filtered_center = kalmanPredictOnly(current_timestamp);
      lost_count_++;
      if (lost_count_ > Params::loss_thres)
      {
        kf_initialized = false;
        RCLCPP_WARN(this->get_logger(), "Target lost for too long, Kalman reset");
      }
    }
  }

  if (filtered_center.x != -1)
  {
    cv::circle(viz, filtered_center, 5, cv::Scalar(0, 0, 255), -1);

    std_msgs::msg::Float64 dyaw_msg;
    dyaw_msg.data = filtered_center.x - viz.cols / 2.0; // 以图像中心为基准，左负右正

    cv::putText(viz, "dyaw: " + std::to_string(filtered_center.x), cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

    dyaw_msg.data *= Params::dyaw_factor;
    dyaw_pub_->publish(dyaw_msg);
  }

  auto end = std::chrono::steady_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  cv::putText(viz, std::to_string(duration) + "ms", cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

  cv::line(viz, cv::Point(viz.cols / 2, 0), cv::Point(viz.cols / 2, viz.rows),
           cv::Scalar(0, 0, 255), 2);

  res_pub_.publish(cv_bridge::CvImage(header, "bgr8", viz).toImageMsg());
}

void HikCameraNode ::init()
{
  RCLCPP_INFO(this->get_logger(), "Detector Node Initialized");
  kalmanInit();
  declareParams();
  createDebugPub();

  // pub
  dyaw_pub_ = this->template create_publisher<std_msgs::msg::Float64>("dyaw", 10);

  // sub
  param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  event_callback_handle_ = param_sub_->add_parameter_event_callback(
      std::bind(&HikCameraNode::paramsEventCB, this, std::placeholders::_1));

  yolo_openvino.yolov5_compiled(Params::weight_path, model);
}

void HikCameraNode::declareParams()
{
  for (auto &param : Params::int_params)
  {
    this->declare_parameter(param.name, param.default_value);
    param.storage_ref = this->get_parameter(param.name).get_value<int>();
  }
  for (auto &param : Params::double_params)
  {
    this->declare_parameter(param.name, param.default_value);
    param.storage_ref = this->get_parameter(param.name).get_value<double>();
  }
  for (auto &param : Params::bool_params)
  {
    this->declare_parameter(param.name, param.default_value);
    param.storage_ref = this->get_parameter(param.name).get_value<bool>();
  }
  for (auto &param : Params::string_params)
  {
    this->declare_parameter(param.name, param.default_value);
    param.storage_ref = this->get_parameter(param.name).get_value<std::string>();
  }
  // for (auto& param : Params::int_vec_params) {
  //     this->declare_parameter(param.name, param.default_value);
  // }
  // for (auto& param : Params::double_vec_params) {
  //     this->declare_parameter(param.name, param.default_value);
  // }
  // for (auto& param : Params::string_vec_params) {
  //     this->declare_parameter(param.name, param.default_value);
  // }
  // for (auto& param : Params::bool_vec_params) {
  //     this->declare_parameter(param.name, param.default_value);
  // }
}

void HikCameraNode::paramsEventCB(const rcl_interfaces::msg::ParameterEvent &event)
{
  for (const auto &changed_param : event.changed_parameters)
  {
    auto int_it = std::find_if(Params::int_params.begin(), Params::int_params.end(),
                               [&changed_param](const auto &param)
                               { return param.name == changed_param.name; });
    if (int_it != Params::int_params.end())
    {
      int_it->storage_ref = changed_param.value.integer_value;
    }
    auto double_it = std::find_if(
        Params::double_params.begin(), Params::double_params.end(),
        [&changed_param](const auto &param) { return param.name == changed_param.name; });
    if (double_it != Params::double_params.end())
    {
      double_it->storage_ref = changed_param.value.double_value;
      RCLCPP_INFO(this->get_logger(), "Parameter %s changed to %f",
                  changed_param.name.c_str(), changed_param.value.double_value);
    }
    auto bool_it = std::find_if(Params::bool_params.begin(), Params::bool_params.end(),
                                [&changed_param](const auto &param)
                                { return param.name == changed_param.name; });
    if (bool_it != Params::bool_params.end())
    {
      bool_it->storage_ref = changed_param.value.bool_value;
    }
    auto string_it = std::find_if(
        Params::string_params.begin(), Params::string_params.end(),
        [&changed_param](const auto &param) { return param.name == changed_param.name; });
    if (string_it != Params::string_params.end())
    {
      string_it->storage_ref = changed_param.value.string_value;
    }
  }
}

cv::Point2f HikCameraNode::detectYOLO(cv::Mat &origin, cv::Mat &viz, float size = 640.0)
{
  cv::Point2f target(-1, -1);
  if (origin.empty())
    return target;

  std::vector<cv::Rect> output_box;
  std::vector<float> output_confidence;
  std::vector<Detection> candidates;

  yolo_openvino.yolov5_detector(model, origin, viz, output_box, output_confidence, size,
                                Params::conf_thres, Params::nms_thres);

  for (size_t i = 0; i < output_box.size(); ++i)
  {
    const auto &box = output_box[i];
    std::vector<std::vector<cv::Point>> contours_;

    cv::Rect safe_box = box & cv::Rect(0, 0, origin.cols, origin.rows);
    if (safe_box.area() == 0)
      continue;
    cv::Mat roi = origin(safe_box);

    cv::Mat channels[3];
    cv::split(roi, channels); // BGR
    cv::Mat g_16, r_16, b_16;
    channels[1].convertTo(g_16, CV_16S);
    channels[2].convertTo(r_16, CV_16S);
    channels[0].convertTo(b_16, CV_16S);

    // Step 1: 找亮区域 (bit 0)
    cv::Mat final_mask;
    if (Params::filter_mode & 1)
    {
      cv::Mat gray;
      cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
      cv::threshold(gray, final_mask, Params::binary_thres, 255, cv::THRESH_BINARY);
    }
    else
    {
      final_mask = cv::Mat::ones(roi.size(), CV_8U) * 255;
    }

    // Step 2: 排除过曝像素 (bit 1)
    if (Params::filter_mode & 2)
    {
      cv::Mat overexp_mask;
      cv::bitwise_and(channels[2] > Params::overexp_thres,
                      channels[1] > Params::overexp_thres, overexp_mask);
      cv::bitwise_and(overexp_mask, channels[0] > Params::overexp_thres, overexp_mask);
      cv::Mat not_overexp;
      cv::bitwise_not(overexp_mask, not_overexp);
      cv::bitwise_and(final_mask, not_overexp, final_mask);
    }

    // Step 3: ExG 像素级绿色判定 (bit 2)
    if (Params::filter_mode & 4)
    {
      cv::Mat exg_mask = ((2 * g_16 - r_16 - b_16) > Params::exg_thres);
      cv::bitwise_and(final_mask, exg_mask, final_mask);
    }

    // 计算 g_ratio 用于多目标排序（整体特征）
    int mean_g_ratio = 0;
    {
      double mean_g = cv::mean(g_16)[0];
      double mean_sum = cv::mean(g_16)[0] + cv::mean(r_16)[0] + cv::mean(b_16)[0];
      if (mean_sum < 1)
        mean_sum = 1;
      mean_g_ratio = (int)(mean_g * 100.0 / mean_sum);
    }

    double green_pixels = cv::countNonZero(final_mask);
    double total_pixels = roi.rows * roi.cols;
    double ratio = (total_pixels > 0) ? (green_pixels / total_pixels) : 0;

    // 发布 debug 图像（无论是否通过占比检查都发布）
    cv::Mat debug_img;
    cv::cvtColor(final_mask, debug_img, cv::COLOR_GRAY2BGR);
    cv::resize(debug_img, debug_img, cv::Size(200, 200), 0, 0, cv::INTER_NEAREST);
    int y = 12;
    double fs = 0.3;
    cv::Scalar tc(0, 255, 0);
    cv::Scalar rc(0, 0, 255);

    // ROI 实际像素统计
    cv::Scalar mean_bgr = cv::mean(roi);
    double min_g, max_g;
    cv::minMaxLoc(channels[1], &min_g, &max_g);
    int bright_px = cv::countNonZero(final_mask);
    int overexp_px = 0;
    if (Params::filter_mode & 2)
    {
      cv::Mat oe;
      cv::bitwise_and(channels[2] > Params::overexp_thres,
                      channels[1] > Params::overexp_thres, oe);
      cv::bitwise_and(oe, channels[0] > Params::overexp_thres, oe);
      overexp_px = cv::countNonZero(oe);
    }

    cv::putText(debug_img, "mode:" + std::to_string(Params::filter_mode), cv::Point(2, y),
                cv::FONT_HERSHEY_SIMPLEX, fs, tc, 1);
    cv::putText(debug_img,
                "sz:" + std::to_string(roi.cols) + "x" + std::to_string(roi.rows),
                cv::Point(2, y * 2), cv::FONT_HERSHEY_SIMPLEX, fs, tc, 1);
    cv::putText(debug_img,
                "mean B:" + std::to_string((int)mean_bgr[0]) +
                    " G:" + std::to_string((int)mean_bgr[1]) +
                    " R:" + std::to_string((int)mean_bgr[2]),
                cv::Point(2, y * 3), cv::FONT_HERSHEY_SIMPLEX, fs, tc, 1);
    cv::putText(debug_img,
                "G min:" + std::to_string((int)min_g) +
                    " max:" + std::to_string((int)max_g),
                cv::Point(2, y * 4), cv::FONT_HERSHEY_SIMPLEX, fs, tc, 1);
    cv::putText(debug_img, "overexp_px:" + std::to_string(overexp_px),
                cv::Point(2, y * 5), cv::FONT_HERSHEY_SIMPLEX, fs, tc, 1);
    cv::putText(debug_img,
                "final_px:" + std::to_string(bright_px) + "/" +
                    std::to_string((int)total_pixels),
                cv::Point(2, y * 6), cv::FONT_HERSHEY_SIMPLEX, fs, tc, 1);
    cv::putText(debug_img,
                "g_ratio:" + std::to_string(mean_g_ratio) +
                    " ratio:" + std::to_string(ratio).substr(0, 4),
                cv::Point(2, y * 7), cv::FONT_HERSHEY_SIMPLEX, fs,
                (ratio >= Params::green_ratio_thres) ? tc : rc, 1);
    det_pub_.publish(cv_bridge::CvImage(header, "bgr8", debug_img).toImageMsg());

    if (total_pixels == 0 || ratio < Params::green_ratio_thres)
    {
      continue;
    }

    cv::findContours(final_mask, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    contours_.erase(std::remove_if(contours_.begin(), contours_.end(),
                                   [](const std::vector<cv::Point> &contour)
                                   {
                                     double area = cv::contourArea(contour);
                                     double rect_area = cv::boundingRect(contour).area();
                                     if (rect_area == 0)
                                       return true;
                                     double filled_ratio = area / rect_area;
                                     return filled_ratio < Params::filled_ratio;
                                   }),
                    contours_.end());

    if (!contours_.empty())
    {
      auto rect = cv::boundingRect(contours_[0]);
      Detection det;
      det.confidence = output_confidence[i];
      det.box = box;
      det.center =
          cv::Point((rect.tl().x + rect.br().x) / 2, (rect.tl().y + rect.br().y) / 2);
      det.g_ratio = mean_g_ratio;
      candidates.push_back(det);
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Detection &a, const Detection &b) { return a.g_ratio > b.g_ratio; });

  if (candidates.size() > 0)
  {
    auto &best = candidates[0];
    cv::rectangle(viz, best.box, cv::Scalar(0, 255, 255), 2);
    cv::circle(viz, best.center, 5, cv::Scalar(0, 255, 255), -1);

    target.x = best.box.tl().x + best.center.x;
    target.y = best.box.tl().y + best.center.y;

    roi_pub_.publish(
        cv_bridge::CvImage(header, "bgr8",
                           origin(best.box & cv::Rect(0, 0, origin.cols, origin.rows)))
            .toImageMsg());
  }

  return target;
}

void HikCameraNode::kalmanInit()
{
  // 使用CV模型（匀速运动模型），状态向量：[x, y, vx, vy]
  int stateSize = 4; // 状态维度：x, y, vx, vy
  int measSize = 2;  // 测量维度：x, y
  int contrSize = 0; // 控制维度

  kf = cv::KalmanFilter(stateSize, measSize, contrSize, CV_32F);

  // 状态转移矩阵 F (4x4)
  // [1, 0, dt, 0]
  // [0, 1, 0, dt]
  // [0, 0, 1, 0]
  // [0, 0, 0, 1]
  double dt = 0.01; // 100fps，约10ms一帧
  kf.transitionMatrix =
      (cv::Mat_<float>(4, 4) << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1);

  // 测量矩阵 H (2x4)
  kf.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);

  // 过程噪声协方差矩阵 Q (4x4)
  // 位置噪声和速度噪声
  float processNoisePos = 1e-2; // 位置过程噪声
  float processNoiseVel = 1e-2; // 速度过程噪声
  kf.processNoiseCov =
      (cv::Mat_<float>(4, 4) << processNoisePos, 0, 0, 0, 0, processNoisePos, 0, 0, 0, 0,
       processNoiseVel, 0, 0, 0, 0, processNoiseVel);

  // 测量噪声协方差矩阵 R (2x2)
  float measureNoisePos = 0.0001; // 测量噪声，可根据实际情况调整
  kf.measurementNoiseCov =
      (cv::Mat_<float>(2, 2) << measureNoisePos, 0, 0, measureNoisePos);

  // 后验误差协方差矩阵 P (4x4)
  setIdentity(kf.errorCovPost, cv::Scalar::all(1));

  // 初始化状态向量
  kf.statePost = cv::Mat::zeros(stateSize, 1, CV_32F);

  kf_initialized = false;
  measurement = cv::Mat::zeros(measSize, 1, CV_32F);
  last_timestamp = 0.0;
  lost_count_ = 0;
}

cv::Point2f HikCameraNode::kalmanUpdate(cv::Point2f measured_pos,
                                        double current_timestamp)
{
  // 计算时间差
  if (!kf_initialized)
  {
    // 首次检测，初始化卡尔曼滤波器状态
    kf.statePost.at<float>(0) = measured_pos.x;
    kf.statePost.at<float>(1) = measured_pos.y;
    kf.statePost.at<float>(2) = 0; // 初始vx
    kf.statePost.at<float>(3) = 0; // 初始vy
    kf_initialized = true;
    last_timestamp = current_timestamp;
    return measured_pos;
  }

  // 计算时间差，用于更新状态转移矩阵
  double dt = current_timestamp - last_timestamp;
  if (dt > 0)
  {
    // 更新状态转移矩阵中的dt
    kf.transitionMatrix.at<float>(0, 2) = dt;
    kf.transitionMatrix.at<float>(1, 3) = dt;
  }

  // 预测步骤
  cv::Mat prediction = kf.predict();
  cv::Point2f predicted_pos(prediction.at<float>(0), prediction.at<float>(1));

  // 更新测量值
  measurement.at<float>(0) = measured_pos.x;
  measurement.at<float>(1) = measured_pos.y;

  // 更新步骤
  cv::Mat estimated = kf.correct(measurement);
  cv::Point2f filtered_pos(estimated.at<float>(0), estimated.at<float>(1));

  last_timestamp = current_timestamp;

  return filtered_pos;
}

cv::Point2f HikCameraNode::kalmanPredictOnly(double current_timestamp)
{
  // 计算时间差
  double dt = current_timestamp - last_timestamp;
  if (dt > 0.1)
    dt = 0.01; // 限制最大时间差
  if (dt > 0)
  {
    kf.transitionMatrix.at<float>(0, 2) = dt;
    kf.transitionMatrix.at<float>(1, 3) = dt;
  }

  // 预测步骤
  cv::Mat prediction = kf.predict();
  cv::Point2f predicted_pos(prediction.at<float>(0), prediction.at<float>(1));

  last_timestamp = current_timestamp;
  return predicted_pos;
}

void HikCameraNode::createDebugPub()
{
  roi_pub_ = image_transport::create_publisher(this, "detector/roi_image");
  det_pub_ = image_transport::create_publisher(this, "detector/binary_image");
  res_pub_ = image_transport::create_publisher(this, "detector/result_image");
}

void HikCameraNode::destroyDebugPub()
{
  roi_pub_.shutdown();
  det_pub_.shutdown();
  res_pub_.shutdown();
}

/*****************************HIK CAMERA***************************/
HikCameraNode::HikCameraNode(const rclcpp::NodeOptions &options) :
    Node("hik_camera", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

  MV_CC_DEVICE_INFO_LIST device_list;
  // enum device
  nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

  while (device_list.nDeviceNum == 0 && rclcpp::ok())
  {
    RCLCPP_ERROR(this->get_logger(), "No camera found!");
    RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  }

  MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

  MV_CC_OpenDevice(camera_handle_);

  // Get camera information
  MV_CC_GetImageInfo(camera_handle_, &img_info_);
  image_buffer_.resize(img_info_.nHeightMax * img_info_.nWidthMax * 3); // RGB buffer
  // Init convert param
  convert_param_.nWidth = img_info_.nWidthValue;
  convert_param_.nHeight = img_info_.nHeightValue;
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
  convert_param_.pDstBuffer = image_buffer_.data();
  convert_param_.nDstBufferSize = image_buffer_.size();

  declareParameters();
  params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));
  /***Start***/
  init();
  /***End***/
  MV_CC_StartGrabbing(camera_handle_);

  capture_thread_ = std::thread{
      [this]() -> void
      {
        MV_FRAME_OUT out_frame;
        RCLCPP_INFO(this->get_logger(), "Starting image processing with OpenCV!");

        while (rclcpp::ok())
        {
          nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
          if (MV_OK == nRet)
          {
            // 设置转换参数
            convert_param_.pSrcData = out_frame.pBufAddr;
            convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

            // 转换为RGB格式
            nRet = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
            if (MV_OK != nRet)
            {
              RCLCPP_ERROR(this->get_logger(), "Pixel convert failed: 0x%x", nRet);
              MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
              continue;
            }

            // 创建OpenCV Mat
            cv::Mat rgb_image(out_frame.stFrameInfo.nHeight, out_frame.stFrameInfo.nWidth,
                              CV_8UC3, convert_param_.pDstBuffer);

            // 自定义处理代码
            processImage(rgb_image);

            MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
            fail_count_ = 0;
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_StartGrabbing(camera_handle_);
            fail_count_++;
          }

          if (fail_count_ > 5)
          {
            RCLCPP_FATAL(this->get_logger(), "Camera failed!");
            rclcpp::shutdown();
          }
        }
      }};
}

HikCameraNode ::~HikCameraNode()
{
  destroyDebugPub();
  if (capture_thread_.joinable())
  {
    capture_thread_.join();
  }
  if (camera_handle_)
  {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
  }
  RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
}

rcl_interfaces::msg::SetParametersResult
HikCameraNode ::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters)
  {
    if (param.get_name() == "exposure_time")
    {
      int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
      if (MV_OK != status)
      {
        result.successful = false;
        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
      }
    }
    else if (param.get_name() == "gain")
    {
      int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      if (MV_OK != status)
      {
        result.successful = false;
        result.reason = "Failed to set gain, status = " + std::to_string(status);
      }
    }
  }
  return result;
}

void HikCameraNode ::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  MVCC_FLOATVALUE f_value;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  // Exposure time
  param_desc.description = "Exposure time in microseconds";
  MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
  RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

  // Gain
  param_desc.description = "Gain";
  MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
  RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
}
/*****************************HIK CAMERA***************************/
} // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
