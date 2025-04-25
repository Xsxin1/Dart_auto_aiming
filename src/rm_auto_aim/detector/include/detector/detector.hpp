#ifndef DETECTOR__DETECTOR_HPP_
#define DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/debug_lights.hpp"
#include "detector/light.hpp"

namespace rm_auto_aim
{
class Detector
{
public:
  struct LightParams
  {
    int binaryThreshold;
    int epsilon;
    int contoursRatioMin;  // width/height 100x
    int contoursRatioMax;  // width/height 100x
    int contoursAreaMin;   // width*height
    int contoursAreaMax;   // width*height

    int filledRatioMin;   // filled/area 0-100 100x
    int filledRatioMax;   // filled/area 0-100 100x
    int ellipseAreaMin;   // width*height
    int ellipseAreaMax;   // width*height
    int ellipseRatioMin;  // width/height 100x
    int ellipseRatioMax;  // width/height 100x
    int detectColor;      // 0-RED, 1-BLUE
  };

  Detector(LightParams & light, bool cv_debug = false, bool fg_debug = false);

  float filledOfEllipse(const cv::Mat & input, const cv::RotatedRect & ellipse_input);

  std::vector<std::vector<cv::Point>> adaptiveFindInnerContours(
    const cv::Mat & imgBinary, int areaThreshold = 100, bool returnOutmost = false);

  std::vector<Light> detect(const cv::Mat & input);

  cv::Mat preprocessImage(const cv::Mat & input);

  std::vector<Light> findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img);

  std::vector<Light> matchLights(std::vector<Light> & lights);

  // For debug usage
  void drawResults(cv::Mat & img);

  LightParams l;

  // Debug msgs
  bool cv_debug_;
  bool fg_debug_;
  cv::Mat binary_img;
  auto_aim_interfaces::msg::DebugLights debug_lights_msg_;

private:
  bool isLight(const Light & possible_light);
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  LightType isLight(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
};

}  // namespace rm_auto_aim

#endif  // DETECTOR__DETECTOR_HPP_
