// MV-CS060-10UC-PRO IMX178 3072×2048 59.6fps USB3.0

#include "detector/detector.hpp"

namespace rm_auto_aim
{
Detector::Detector(LightParams & light, bool cv_debug, bool fg_debug)
: l(light), cv_debug_(cv_debug), fg_debug_(fg_debug)
{
  if (cv_debug_) {
    // 创建窗口
    cv::namedWindow("DebugWin", cv::WindowFlags::WINDOW_AUTOSIZE);
    cv::createTrackbar("BinaryThreshold", "DebugWin", &l.binaryThreshold, 255);
    cv::createTrackbar("1/epsilon", "DebugWin", &l.epsilon, 10000);
    cv::createTrackbar("contoursRatioMin10x", "DebugWin", &l.contoursRatioMin, 200);
    cv::createTrackbar("contoursRatioMax10x", "DebugWin", &l.contoursRatioMax, 200);
    cv::createTrackbar("contoursAreaMax/100", "DebugWin", &l.contoursAreaMax, 15520);
    cv::createTrackbar("contoursAreaMin", "DebugWin", &l.contoursAreaMin, 15520);
    cv::createTrackbar("EllipseRatioMax100x", "DebugWin", &l.ellipseRatioMax, 200);
    cv::createTrackbar("EllipseRatioMin100x", "DebugWin", &l.ellipseRatioMin, 200);
    cv::createTrackbar("filledRatioMin100x", "DebugWin", &l.filledRatioMin, 100);
    cv::createTrackbar("filledRatioMax100x", "DebugWin", &l.filledRatioMax, 100);
    cv::createTrackbar("EllipseAreaMin", "DebugWin", &l.ellipseAreaMin, 15520);
  }
}

std::vector<Light> Detector::detect(const cv::Mat & input)
{
  binary_img = preprocessImage(input);
  lights_ = findLights(input, binary_img);
  return lights_;
}

cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)
{
  cv::Mat binary_img;
  std::vector<cv::Mat> channels;
  split(rgb_img, channels);
  binary_img = channels[1] - channels[2];
  cv::threshold(binary_img, binary_img, l.binaryThreshold, 255, cv::THRESH_BINARY);

  if (cv_debug_) {
    cv::Mat vis_binary;
    cv::resize(binary_img, vis_binary, cv::Size(1280, 720));
    cv::imshow("vis_binary", vis_binary);
  }

  return binary_img;
}

std::vector<Light> Detector::findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<Light> lights;
  debug_lights_msg_.data.clear();

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  vector<vector<cv::Point>> contours = adaptiveFindInnerContours(binary_img, 100);

  // approx contours
  vector<vector<cv::Point>> approxContours;

  for (size_t i = 0; i < contours.size(); i++) {
    vector<cv::Point> approx;
    auto_aim_interfaces::msg::DebugLight debug_light;

    cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true) / 1.0 / l.epsilon, true);
    cv::Rect tempRetangle = cv::boundingRect(approx);

    if (  // tl and br are the top left and bottom right points of screen
      tempRetangle.area() > l.contoursAreaMin && tempRetangle.area() < l.contoursAreaMax * 100 &&
      tempRetangle.tl().y > 0 && tempRetangle.br().y < binary_img.rows && tempRetangle.tl().x > 0 &&
      tempRetangle.br().x < binary_img.cols) {
      approxContours.emplace_back(approx);

      // fill debug light
      if (fg_debug_) {
        geometry_msgs::msg::Pose2D point;
        point.x = tempRetangle.br().x;
        point.y = tempRetangle.br().y;
        debug_light.corners.emplace_back(point);

        point.x = tempRetangle.tl().x;
        point.y = tempRetangle.tl().y;
        debug_light.corners.emplace_back(point);

        debug_light.contour_area = tempRetangle.area();

        debug_light.contour_width = tempRetangle.width;
        debug_light.contour_height = tempRetangle.height;
        debug_lights_msg_.data.emplace_back(debug_light);
      }
    }
  }

  if (cv_debug_) {
    cv::Mat vis_contour = rgb_img.clone();
    cv::drawContours(vis_contour, approxContours, -1, cv::Scalar(0, 0, 255), 6);
    cv::resize(vis_contour, vis_contour, cv::Size(1280, 720));
    cv::imshow("vis_contour", vis_contour);
  }

  // fit ellipse
  std::vector<cv::RotatedRect> minEllipse;
  std::vector<std::vector<cv::Point>> ellipsePoints;
  lights.clear();
  for (size_t i = 0; i < approxContours.size(); i++) {
    cv::RotatedRect tempEllipse = cv::fitEllipse(approxContours[i]);

    if (fg_debug_) {
      // fill debug light
      geometry_msgs::msg::Pose2D point;
      point.x = tempEllipse.center.x;
      point.y = tempEllipse.center.y;
      debug_lights_msg_.data[i].ellipse_center = point;
      debug_lights_msg_.data[i].filled = filledOfEllipse(binary_img, tempEllipse);
      debug_lights_msg_.data[i].ellipse_width = tempEllipse.size.width;
      debug_lights_msg_.data[i].ellipse_height = tempEllipse.size.height;
      debug_lights_msg_.data[i].ellipse_area = tempEllipse.size.area();
    }

    if (
      tempEllipse.size.width / tempEllipse.size.height > l.ellipseRatioMin / 100.0 &&
      tempEllipse.size.width / tempEllipse.size.height < l.ellipseRatioMax / 100.0 &&
      filledOfEllipse(binary_img, tempEllipse) < l.filledRatioMax / 100.0 &&
      tempEllipse.size.area() > l.ellipseAreaMin) {
      lights.emplace_back(Light(tempEllipse, approxContours[i]));
    }
  }

  return lights;
}

// TO DO:add Light detector to adjust the light
bool Detector::isLight(const Light & light)
{
  bool is_light;
  return is_light = true;
}

// TODO:to filter the noise
std::vector<Light> Detector::matchLights(std::vector<Light> & lights)
{
  // this->debug_Lights.data.clear();

  if (lights.size() == 2) {
    if (cv::pointPolygonTest(lights[0].ellipsePoints, lights[1].center, false) == 1) {
      if (lights[0].size.area() > lights[1].size.area()) {
        lights.erase(lights.begin());
      } else if (lights[0].size.area() < lights[1].size.area()) {
        lights.erase(lights.begin() + 1);
      }
    }
  }

  return lights;
}

void Detector::drawResults(cv::Mat & img)
{
  // Draw Lights
  for (const auto & light : lights_) {
    cv::ellipse(img, light, cv::Scalar(0, 0, 255), 8);
    cv::circle(img, light.center, 5, cv::Scalar(0, 0, 255), 6);
  }
}

float Detector::filledOfEllipse(const cv::Mat & input, const cv::RotatedRect & ellipse_input)
{
  CV_Assert(input.type() == CV_8UC1);
  if (
    ellipse_input.center.x < 0 || ellipse_input.center.x >= input.cols ||
    ellipse_input.center.y < 0 || ellipse_input.center.y >= input.rows) {
    // std::cerr << "Center point is out of input image bounds!" <<
    // std::endl;
    return -1;
  }

  if (ellipse_input.size.width > 10 || ellipse_input.size.height > 10) {
    // imshow("input", input);
    // cout << "x" << ellipse_input.center.x << " y" <<
    // ellipse_input.center.y << endl;
    cv::Mat mask = cv::Mat::zeros(input.size(), CV_8UC1);
    ellipse(mask, ellipse_input, cv::Scalar(255), -1);
    bitwise_and(input, mask, mask);
    // imshow("mask", mask);
    cv::Scalar gray_sum = cv::sum(mask) / 255.0;
    float ellipse_area =
      CV_PI * (ellipse_input.size.width / 2.0) * (ellipse_input.size.height / 2.0);

    // cout << "gray_sum: " << gray_sum[0] << endl;
    // cout << "ellipse_area: " << ellipse_area << endl;
    return gray_sum[0] / ellipse_area;
  }
  return -1;
}

std::vector<std::vector<cv::Point>> Detector::adaptiveFindInnerContours(
  const cv::Mat & imgBinary, int areaThreshold, bool returnOutmost)
{
  std::vector<std::vector<cv::Point>> contours;
  std::vector<std::vector<cv::Point>> inner_contours;
  std::vector<cv::Vec4i> hierarchy;
  bool haveInnerContours = false;
  findContours(imgBinary, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
  for (size_t i = 0; i < contours.size(); i++) {
    if (
      hierarchy[i][3] != -1 &&
      cv::boundingRect(contours[i]).area() > areaThreshold)  // isn't a outmost contour
    {
      inner_contours.emplace_back(contours[i]);
      haveInnerContours = true;
    }
  }
  if (!haveInnerContours && returnOutmost)
    return contours;
  else
    return inner_contours;
}

}  // namespace rm_auto_aim
