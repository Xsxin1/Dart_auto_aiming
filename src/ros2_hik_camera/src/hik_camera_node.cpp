#include "ros2_hik_camera/hik_camera_node.h"

namespace hik_camera
{

void HikCameraNode ::processImage(cv::Mat & rgb_image)
{
  auto start = std::chrono::steady_clock::now();

  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "cam";

  cv::Mat image;
  cv::cvtColor(rgb_image, image, cv::COLOR_RGB2BGR);

  auto target = detectYOLO(image, 320);

  if (target.area() > 0) {
    cv::rectangle(image, target, cv::Scalar(0, 255, 0), 2);
    std::vector<std::vector<cv::Point>> contours_;

    cv::Mat roi = image(target);

    roi_pub_.publish(cv_bridge::CvImage(header, "bgr8", roi).toImageMsg());
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::findContours(gray, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::remove_if(contours_.begin(), contours_.end(), [](const std::vector<cv::Point> & contour) {
      return cv::contourArea(contour) < 10;
    });

    if (contours_.size() == 1) {
      auto bound_rect = cv::boundingRect(contours_[0]);

      std_msgs::msg::Float64 dyaw_msg;
      dyaw_msg.data =
        (target.tl().x - image.cols / 2 + (bound_rect.tl().x + bound_rect.br().x) / 2) ;

      cv::putText(
        image, "dyaw: " + std::to_string(dyaw_msg.data), cv::Point(10, 60),
        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

      dyaw_msg.data *= Params::dyaw_factor;
      dyaw_pub_->publish(dyaw_msg);
    }
  }

  //   auto aiming = tracker(target, image, delay);

  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  cv::putText(
    image, std::to_string(duration) + "ms", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
    cv::Scalar(0, 255, 0), 2);

  cv::line(
    image, cv::Point(image.cols / 2, 0), cv::Point(image.cols / 2, image.rows),
    cv::Scalar(0, 0, 255), 2);

  res_pub_.publish(cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
}

void HikCameraNode ::init()
{
  RCLCPP_INFO(this->get_logger(), "Detector Node Initialized");
  // kalmanInit();
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

// void HikCameraNode ::kalmanInit()
// {                                   // kalman filter（CV MODEL）
//   kf = KalmanFilter(9, 3, 0);       //[x,y,z,vx,vy,vz,ax,ay,az]
//   meas = Mat::zeros(3, 1, CV_32F);  //[x,y,z]
//   // 状态转移矩阵 (9x9)
//   double dt = 0.01;
//   double dt2 = dt * dt;
//   double dt3 = dt * dt * dt;
//   double dt4 = dt * dt * dt * dt;
//   kf.transitionMatrix =
//     (Mat_<float>(9, 9) << 1, 0, 0, dt, 0, 0, 0.5 * dt2, 0, 0,  // x = x + vx*dt+ 0.5*dt^2*ax
//      0, 1, 0, 0, dt, 0, 0, 0.5 * dt2, 0,                       // y = y + vy*dt+ 0.5*dt^2*ay
//      0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt2,                       // z = z + vz*dt+ 0.5*dt^2*az
//      0, 0, 0, 1, 0, 0, dt, 0, 0,                               // vx = vx + ax*dt
//      0, 0, 0, 0, 1, 0, 0, dt, 0,                               // vy = vy + ay*dt
//      0, 0, 0, 0, 0, 1, 0, 0, dt,                               // vz = vz + az*dt
//      0, 0, 0, 0, 0, 0, 1, 0, 0,                                // ax = ax
//      0, 0, 0, 0, 0, 0, 0, 1, 0,                                // ay = ay
//      0, 0, 0, 0, 0, 0, 0, 0, 1);                               // az = az

//   // 测量矩阵 (3x9: 只观测位置)
//   kf.measurementMatrix =
//     (Mat_<float>(3, 9) << 1, 0, 0, 0, 0, 0, 0, 0, 0,  // 测量x
//      0, 1, 0, 0, 0, 0, 0, 0, 0,                       // 测量y
//      0, 0, 1, 0, 0, 0, 0, 0, 0);                      // 测量z

//   // 过程噪声协方差 (9x9)
//   kf.processNoiseCov =
//     (Mat_<float>(9, 9) << 0.25 * dt4, 0, 0, 0.5 * dt3, 0, 0, 0.5 * dt2, 0, 0,  //
//      0, 0.25 * dt4, 0, 0, 0.5 * dt3, 0, 0, 0.5 * dt2, 0,                       //
//      0, 0, 0.25 * dt4, 0, 0, 0.5 * dt3, 0, 0, 0.5 * dt2,                       //
//      0.5 * dt3, 0, 0, dt2, 0, 0, dt, 0, 0,                                     //
//      0, 0.5 * dt3, 0, 0, dt2, 0, 0, dt, 0,                                     //
//      0, 0, 0.5 * dt3, 0, 0, dt2, 0, 0, dt,                                     //
//      0.5 * dt2, 0, 0, dt, 0, 0, 1, 0, 0,                                       //
//      0, 0.5 * dt2, 0, 0, dt, 0, 0, 1, 0,                                       //
//      0, 0, 0.5 * dt2, 0, 0, dt, 0, 0, 1);                                      //
//   // 测量噪声协方差 (3x3)
//   float r = 0.1;
//   kf.measurementNoiseCov =
//     (Mat_<float>(3, 3) << r, 0, 0,  //1
//      0, r, 0,                       //2
//      0, 0, r);

//   // 后验误差协方差初始化 (9x9单位矩阵)
//   setIdentity(kf.errorCovPost, Scalar::all(1));

//   // 初始化状态 (9x1向量)
//   randn(kf.statePost, Scalar::all(0), Scalar::all(q));
// }

void HikCameraNode::declareParams()
{
  for (auto & param : Params::int_params) {
    this->declare_parameter(param.name, param.default_value);
    param.storage_ref = this->get_parameter(param.name).get_value<int>();
  }
  for (auto & param : Params::double_params) {
    this->declare_parameter(param.name, param.default_value);
    param.storage_ref = this->get_parameter(param.name).get_value<double>();
  }
  for (auto & param : Params::bool_params) {
    this->declare_parameter(param.name, param.default_value);
    param.storage_ref = this->get_parameter(param.name).get_value<bool>();
  }
  for (auto & param : Params::string_params) {
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

void HikCameraNode::paramsEventCB(const rcl_interfaces::msg::ParameterEvent & event)
{
  for (const auto & changed_param : event.changed_parameters) {
    auto int_it = std::find_if(
      Params::int_params.begin(), Params::int_params.end(),
      [&changed_param](const auto & param) { return param.name == changed_param.name; });
    if (int_it != Params::int_params.end()) {
      int_it->storage_ref = changed_param.value.integer_value;
    }
    auto double_it = std::find_if(
      Params::double_params.begin(), Params::double_params.end(),
      [&changed_param](const auto & param) { return param.name == changed_param.name; });
    if (double_it != Params::double_params.end()) {
      double_it->storage_ref = changed_param.value.double_value;
      RCLCPP_INFO(
        this->get_logger(), "Parameter %s changed to %f", changed_param.name.c_str(),
        changed_param.value.double_value);
    }
    auto bool_it = std::find_if(
      Params::bool_params.begin(), Params::bool_params.end(),
      [&changed_param](const auto & param) { return param.name == changed_param.name; });
    if (bool_it != Params::bool_params.end()) {
      bool_it->storage_ref = changed_param.value.bool_value;
    }
    auto string_it = std::find_if(
      Params::string_params.begin(), Params::string_params.end(),
      [&changed_param](const auto & param) { return param.name == changed_param.name; });
    if (string_it != Params::string_params.end()) {
      string_it->storage_ref = changed_param.value.string_value;
    }
  }
}

cv::Rect HikCameraNode::detectYOLO(cv::Mat & origin, float size = 640.0)
{
  cv::Rect target;
  if (origin.empty()) return target;

  Mat viz;
  vector<cv::Rect> output_box;

  target = yolo_openvino.yolov5_detector(
    model, origin, viz, output_box, size, Params::conf_thres, Params::nms_thres);

  return target;
}

// cv::Point HikCameraNode::tracker(cv::Rect & rect, cv::Mat & origin, float delay_time)
// {
//   const float armor_height = 0.07;   // unit:m
//   const float laser_height = 0.026;  // unit:m
//   double meas_yaw, meas_pitch;
//   double pre_yaw, pre_pitch;
//   float X, Y, Z;

//   // const float serial_delay = 0.1;
//   // const float ros_delay = 0.00;
//   double dt = delay_time;
//   double dt2 = dt * dt;
//   double dt3 = dt * dt * dt;
//   double dt4 = dt * dt * dt * dt;
//   kf.transitionMatrix =
//     (Mat_<float>(9, 9) << 1, 0, 0, dt, 0, 0, 0.5 * dt2, 0, 0,  // x = x + vx*dt+ 0.5*dt^2*ax
//      0, 1, 0, 0, dt, 0, 0, 0.5 * dt2, 0,                       // y = y + vy*dt+ 0.5*dt^2*ay
//      0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt2,                       // z = z + vz*dt+ 0.5*dt^2*az
//      0, 0, 0, 1, 0, 0, dt, 0, 0,                               // vx = vx + ax*dt
//      0, 0, 0, 0, 1, 0, 0, dt, 0,                               // vy = vy + ay*dt
//      0, 0, 0, 0, 0, 1, 0, 0, dt,                               // vz = vz + az*dt
//      0, 0, 0, 0, 0, 0, 1, 0, 0,                                // ax = ax
//      0, 0, 0, 0, 0, 0, 0, 1, 0,                                // ay = ay
//      0, 0, 0, 0, 0, 0, 0, 0, 1);                               // az = az

//   kf.processNoiseCov =
//     (Mat_<float>(9, 9) << 0.25 * dt4, 0, 0, 0.5 * dt3, 0, 0, 0.5 * dt2, 0, 0,  //
//      0, 0.25 * dt4, 0, 0, 0.5 * dt3, 0, 0, 0.5 * dt2, 0,                       //
//      0, 0, 0.25 * dt4, 0, 0, 0.5 * dt3, 0, 0, 0.5 * dt2,                       //
//      0.5 * dt3, 0, 0, dt2, 0, 0, dt, 0, 0,                                     //
//      0, 0.5 * dt3, 0, 0, dt2, 0, 0, dt, 0,                                     //
//      0, 0, 0.5 * dt3, 0, 0, dt2, 0, 0, dt,                                     //
//      0.5 * dt2, 0, 0, dt, 0, 0, 1, 0, 0,                                       //
//      0, 0.5 * dt2, 0, 0, dt, 0, 0, 1, 0,                                       //
//      0, 0, 0.5 * dt2, 0, 0, dt, 0, 0, 1);

//   bool has_armor = rect.area() > 0;
//   // check hit
//   bool in_rect = false;
//   static bool initialized = false;
//   static int lost_counter = 0;
//   static std::chrono::_V2::steady_clock::time_point pre, cur;  // unit: ms

//   // 卡尔曼预测（先predict，后correct）
//   // float r =
//   //   sqrt(pow(kf.statePre.at<float>(3), 2) + pow(kf.statePre.at<float>(4), 2)) > 0.5 ? r_max : r_min;
//   // cout << "speed: " << sqrt(pow(kf.statePre.at<float>(3), 2) + pow(kf.statePre.at<float>(4), 2))
//   //      << "R:" << r << endl;

//   kf.measurementNoiseCov.at<float>(0, 0) = r_max;
//   kf.measurementNoiseCov.at<float>(1, 1) = r_max;
//   kf.measurementNoiseCov.at<float>(2, 2) = r_max;

//   cv::Mat state = kf.predict();
//   float pred_x = state.at<float>(0);
//   float pred_y = state.at<float>(1);
//   float pred_z = state.at<float>(2);

//   if (has_armor) {
//     cv::Rect detect_rect = cv::Rect(
//       rect.x + rect.width * 0.05, rect.y + rect.height * 0.142, rect.width * 0.9,
//       rect.height * 0.714);
//     rectangle(origin, detect_rect, cv::Scalar(0, 0, 255), 2);
//     in_rect = detect_rect.contains(cv::Point(cx, cy));

//     Z = fy * armor_height / rect.height;
//     if (Z > 5.6 || Z < 0.1) {
//       initialized = false;
//       return cv::Point(0, 0);
//     }
//     X = (rect.x + rect.width / 2.0 - cx) / fx * Z;
//     Y = (rect.y + rect.height / 2.0 - cy) / fy * Z;
//     meas.at<float>(0) = X;
//     meas.at<float>(1) = Y;
//     meas.at<float>(2) = Z;
//     if (state.at<float>(0) + state.at<float>(1) - meas.at<float>(0) - meas.at<float>(1) > 2) {
//       has_armor = false;
//     }
//   }

//   // update kalman state
//   if (!initialized && has_armor) {
//     // stateVector: [x, y, z, vx, vy ,vz]
//     kf.statePost.at<float>(0) = X;
//     kf.statePost.at<float>(1) = Y;
//     kf.statePost.at<float>(2) = Z;
//     kf.statePost.at<float>(3) = 0;
//     kf.statePost.at<float>(4) = 0;
//     kf.statePost.at<float>(5) = 0;
//     kf.statePost.at<float>(6) = 0;
//     kf.statePost.at<float>(7) = 0;
//     kf.statePost.at<float>(8) = 0;
//     initialized = true;
//     lost_counter = 0;
//     pre = std::chrono::steady_clock::now();
//     cur = std::chrono::steady_clock::now();
//   }
//   // else if (initialized && has_armor && pred_y > 0) {
//   //   kf.statePost.at<float>(3) = 0;
//   //   kf.statePost.at<float>(4) = 0;
//   //   kf.statePost.at<float>(5) = 0;
//   //   kf.statePost.at<float>(6) = 0;
//   //   kf.statePost.at<float>(7) = 0;
//   //   kf.statePost.at<float>(8) = 0;
//   // }

//   // updata track state
//   if (!has_armor) {
//     lost_counter++;
//     if (lost_counter > max_lost_frames) {
//       initialized = false;
//       const float lost_gain = exp(-lost_counter * 0.5f);  // 指数衰减
//       kf.statePost.at<float>(3) *= lost_gain;
//       kf.statePost.at<float>(4) *= lost_gain;
//       kf.statePost.at<float>(5) *= lost_gain;
//       setIdentity(kf.errorCovPost, Scalar::all(1.0));  // 重置不确定性
//       // RCLCPP_WARN_STREAM(this->get_logger(), "Armor lost");
//     }
//   } else {
//     state = kf.correct(meas);
//     pred_x = state.at<float>(0);
//     pred_y = state.at<float>(1);
//     pred_z = state.at<float>(2);
//     RCLCPP_DEBUG_STREAM(
//       this->get_logger(), "Armor prediction: " << pred_x << ", " << pred_y << ", " << pred_z);
//     lost_counter = 0;

//     // predict
//     Mat savedState = kf.statePost.clone();
//     Mat savedCov = kf.errorCovPost.clone();

//     for (int j = 0; j < pre_step; j++) {
//       Mat predictState = kf.transitionMatrix * savedState;
//       Mat predictCov =
//         kf.transitionMatrix * savedCov * kf.transitionMatrix.t() + kf.processNoiseCov;

//       circle(
//         origin,
//         cv::Point2f(
//           fx * tan(predictState.at<float>(0) / predictState.at<float>(2)) + cx,
//           fy * tan(predictState.at<float>(1) / predictState.at<float>(2)) + cy),
//         2, cv::Scalar(0, 0, 255));
//       // cout << "predict:" << fx * tan(predictState.at<float>(0) / predictState.at<float>(2)) + cx
//       //      << ", " << fy * tan(predictState.at<float>(1) / predictState.at<float>(2)) + cy << endl;

//       savedState = predictState;
//       savedCov = predictCov;
//     }
//     pred_x = savedState.at<float>(0);
//     pred_y = savedState.at<float>(1);
//     pred_z = savedState.at<float>(2);

//     std_msgs::msg::Float32 gimbal_yaw_i_msg;
//     rclcpp::Time current_time = this->now();

//     rclcpp::Duration image_delay = current_time - image_time_;
//     double image_delay_second = image_delay.seconds();

//     rclcpp::Duration serial_delay(rclcpp::Duration::from_seconds(delay_test));
//     rclcpp::Time last_time_ = image_time_ - serial_delay;
//     rclcpp::Duration total_time = current_time - last_time_;

//     gimbal_yaw_i_msg.data = calculateRotation(last_time_, current_time);
//     debug_pub_->publish(gimbal_yaw_i_msg);

//     RCLCPP_DEBUG_STREAM(this->get_logger(), "Time delay: " << total_time.seconds() << "s");

//     pre_yaw = -atan2((pred_x + state.at<float>(3) * delay_time), pred_z) -
//               gimbal_yaw_i_msg.data * delay_gain;

//     pre_pitch = atan2((pred_y + pitch_offset + pitch_offset_gain * pred_z), pred_z);

//     // auto [meas_yaw, meas_pitch] = pixel2angle((rect.br() + rect.tl()) / 2);
//     // auto laser_pitch = compute_laser_pitch(armor_height, rect.height, laser_height);
//     // pred_z -= laser_pitch;

//     // RCLCPP_DEBUG_STREAM(
//     //   this->get_logger(), "meas p:" << meas_pitch << " a_p" << laser_pitch << " y" << meas_yaw);
//   }

//   // adjust hitted
//   static int lost_hit_counter = 0;
//   if (in_rect) {
//     cur = std::chrono::steady_clock::now();
//     lost_hit_counter = 0;
//   } else {
//     lost_hit_counter++;
//     if (lost_hit_counter > max_lost_hit_frames) pre = std::chrono::steady_clock::now();
//   }
//   // cout << "lost_hit_counter: " << lost_hit_counter << endl;

//   auto hitted_time = duration_cast<milliseconds>(cur - pre).count();  // unit: ms

//   std_msgs::msg::Bool hitted_flag;
//   if (hitted_time > hitted_min_time) {
//     hitted_flag.data = hitted = true;
//   } else {
//     hitted_flag.data = hitted = false;
//   }
//   hitted_pub_->publish(hitted_flag);

//   // RCLCPP_DEBUG(
//   //   this->get_logger(), "Hitted time: %ld ms, Hitted: %s", hitted_time,
//   //   hitted_flag.data ? "true" : "false");

//   // publish
//   if (initialized) {
//     // Publish detected armor
//     geometry_msgs::msg::Point point;
//     point.x = 1;
//     point.y = limit(pre_yaw, -CV_PI / 6, CV_PI / 6) + gimble_yaw;
//     point.z = limit(pre_pitch, -CV_PI / 12, CV_PI / 12) + gimbal_pitch;
//     enemy_dposition_pub_->publish(point);

//     RCLCPP_DEBUG_STREAM(this->get_logger(), "Armor dangle: " << point.y << ", " << point.z);
//     RCLCPP_DEBUG_STREAM(
//       this->get_logger(), "Armor dposition: " << pred_x << ", " << pred_y << ", " << pred_z);
//   }
//   // else {
//   //   // Publish detected armor
//   //   geometry_msgs::msg::Point point;

//   //   point.x = -1;
//   //   point.y = enemy_angle * -gimble_yaw;
//   //   point.z = 0;
//   //   enemy_dposition_pub_->publish(point);
//   //   return cv::Point(0, 0);

//   //   RCLCPP_DEBUG_STREAM(this->get_logger(), "Armor dangle: " << point.y << ", " << point.z);
//   // }

//   return cv::Point2f(fx * tan(pred_x / pred_z) + cx, fy * tan(pred_y / pred_z) + cy);
// }

void HikCameraNode::createDebugPub()
{
  roi_pub_ = image_transport::create_publisher(this, "detector/roi_image");
  det_pub_ = image_transport::create_publisher(this, "detector/detected_image");
  res_pub_ = image_transport::create_publisher(this, "detector/result_image");
}

void HikCameraNode::destroyDebugPub()
{
  roi_pub_.shutdown();
  det_pub_.shutdown();
  res_pub_.shutdown();
}

/*****************************HIK CAMERA***************************/
HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

  MV_CC_DEVICE_INFO_LIST device_list;
  // enum device
  nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

  while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(), "No camera found!");
    RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  }

  MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

  MV_CC_OpenDevice(camera_handle_);

  // Get camera information
  MV_CC_GetImageInfo(camera_handle_, &img_info_);
  image_buffer_.resize(img_info_.nHeightMax * img_info_.nWidthMax * 3);  // RGB buffer
  // Init convert param
  convert_param_.nWidth = img_info_.nWidthValue;
  convert_param_.nHeight = img_info_.nHeightValue;
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
  convert_param_.pDstBuffer = image_buffer_.data();
  convert_param_.nDstBufferSize = image_buffer_.size();

  declareParameters();
  /***Start***/
  init();
  /***End***/
  MV_CC_StartGrabbing(camera_handle_);

  capture_thread_ = std::thread{[this]() -> void {
    MV_FRAME_OUT out_frame;
    RCLCPP_INFO(this->get_logger(), "Starting image processing with OpenCV!");

    while (rclcpp::ok()) {
      nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
      if (MV_OK == nRet) {
        // 设置转换参数
        convert_param_.pSrcData = out_frame.pBufAddr;
        convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
        convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

        // 转换为RGB格式
        nRet = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
        if (MV_OK != nRet) {
          RCLCPP_ERROR(this->get_logger(), "Pixel convert failed: 0x%x", nRet);
          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          continue;
        }

        // 创建OpenCV Mat
        cv::Mat rgb_image(
          out_frame.stFrameInfo.nHeight, out_frame.stFrameInfo.nWidth, CV_8UC3,
          convert_param_.pDstBuffer);

        // 自定义处理代码
        processImage(rgb_image);

        MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
        fail_count_ = 0;
      } else {
        RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_StartGrabbing(camera_handle_);
        fail_count_++;
      }

      if (fail_count_ > 5) {
        RCLCPP_FATAL(this->get_logger(), "Camera failed!");
        rclcpp::shutdown();
      }
    }
  }};
}

HikCameraNode ::~HikCameraNode()
{
  destroyDebugPub();
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (camera_handle_) {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
  }
  RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
}

rcl_interfaces::msg::SetParametersResult HikCameraNode ::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "exposure_time") {
      int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
      if (MV_OK != status) {
        result.successful = false;
        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "gain") {
      int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      if (MV_OK != status) {
        result.successful = false;
        result.reason = "Failed to set gain, status = " + std::to_string(status);
      }
    } else {
      result.successful = false;
      result.reason = "Unknown parameter: " + param.get_name();
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
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
